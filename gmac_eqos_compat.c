/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * 裸机 DWMAC4 / Synopsys EQOS 最小 DMA 收发实现。
 *
 * 为何能对齐 Linux：
 *   寄存器偏移、通道布局、描述符四 dword 格式与 OWN 位语义，与内核
 *   drivers/net/ethernet/stmicro/stmmac 下 dwmac4_dma.h / dwmac4_descs.h /
 *   dwmac4_lib.c 一致，便于用内核 DTS + 驱动反查你 SoC 上的复位/时钟/strap。
 *
 * 调用顺序（必须遵守，否则行为未定义或 DMA 不工作）：
 *   1) 填 hal->csr_base：与 CPU 访问 MAC CSR 的 AXI 窗口基址一致（已含偏移则
 *      不要再加）。未解复位/未供时钟时读寄存器会得到垃圾值或总线挂死。
 *   2) gmac_mac_init(hal)：软复位 DMA、配 MTL/DMA 通道 0、关中断、设过滤与
 *      MAC 模式。此时还不建环、不置 TE/RE（在 new_dma 里才打开）。
 *   3) gmac_esp_new_dma(&handle)：建描述符环、写 ring base/len/tail、启动 ST/SR
 *      与 TE/RE。
 *
 * 环回 / PHY：
 *   默认置 GMAC_CONFIG_LM（MAC 近端环回），帧从 MAC 发端直接回到收端，便于
 *   无 PHY、无对端时验证 CSR 与 DMA。接真实 RGMII/PHY 时应在包含本文件前定义
 *   GMAC_EQOS_NO_MAC_LOOPBACK，并自行 MDIO 协商、清 LM、必要时配 EXT_CONFIG
 *   等（本文件不覆盖 PHY 与 RGMII 时序寄存器）。
 *
 * Cache / 一致性（裸机最常见踩坑）：
 *   描述符与 tx/rx 缓冲若在带 D-cache 的 RAM 中，必须在写环/填包后 clean
 *   descriptor & buffer，在读描述符回写或读包前 invalidate。本实现仅用 dmb()
 *   保证 CPU 次序，不代替 cache 维护。若 DMA 与 CPU 非一致路径，还需按 SoC
 *   手册处理 snoop 或 non-cache 段。
 *
 * 物理地址：
 *   dma_phys() 目前为 (uint32_t)(uintptr_t)p，仅适用于 DMA 可见的低 4G 且与
 *   CPU 同址映射；若需高位地址或 IOMMU，必须改写。
 *
 * Bring-up 日志：
 *   默认 printf；编译 -DGMAC_EQOS_SILENT 全关；或编译前 #define GMAC_BRUP(...)
 *   接到 UART printf。
 */
#include "gmac_eqos_hal.h"

#include <stddef.h>
#include <string.h>

#include "eqos_dwmac4_hw.h"

#if defined(GMAC_EQOS_SILENT)
#define GMAC_BRUP(...) ((void)0)
#elif !defined(GMAC_BRUP)
#include <stdio.h>
#define GMAC_BRUP(...) printf(__VA_ARGS__)
#endif

#ifndef GMAC_EQOS_NO_MAC_LOOPBACK
#define GMAC_EQOS_MAC_LOOPBACK_DEFAULT 1
#else
#define GMAC_EQOS_MAC_LOOPBACK_DEFAULT 0
#endif

/* 环深度：越大越能抗突发，但静态 RAM 占用越大；须与 DMA_CHAN_*_RING_LEN 写入
 * 的 “深度-1” 一致。仅实现单队列 ch0。 */
#define EQOS_RING_N		8u
#define EQOS_CHAN		0u
/* PBL：单次 DMA burst 描述的数据块数相关配置，过大可能超 AXI 从机能力；
 * 过小降低吞吐；8 为常见折中，可按手册与性能再调。 */
#define EQOS_PBL		8u
/* 须 >= 本驱动可能的最大帧长，且与 DMA RBSZ（接收缓冲大小字段）一致；
 * 若硬件/驱动支持巨型帧，须同步增大并检查描述符格式。 */
#define EQOS_RX_BUF_SZ	2048u

#define REG32(b, o) (*(volatile uint32_t *)((uintptr_t)(b) + (uintptr_t)(o)))

struct gmac_esp_dma_state {
	gmac_hal_context_t *hal;
	struct eqos_dma_desc tx_ring[EQOS_RING_N];
	struct eqos_dma_desc rx_ring[EQOS_RING_N];
	uint8_t tx_buf[EQOS_RING_N][EQOS_RX_BUF_SZ];
	uint8_t rx_buf[EQOS_RING_N][EQOS_RX_BUF_SZ];
	unsigned int tx_head;
	unsigned int rx_cons;
	int running;
};

static struct gmac_esp_dma_state s_dma;
static gmac_hal_context_t *s_hal_for_newdma;

static uint32_t dma_phys(const void *p)
{
	return (uint32_t)(uintptr_t)p;
}

/* 编译器屏障：保证描述符内存写入顺序在 DMA 看到 OWN=1 之前完成。
 * 不等于 D-cache flush。 */
static void dmb(void)
{
	__asm__ __volatile__("" ::: "memory");
}

/* DMA 软复位：手册要求上电或异常恢复时先拉 SFT_RESET，待硬件自清 0。
 * 超时说明总线不通、时钟未开、或 IP 未解复位。 */
static int dma_soft_reset(uintptr_t csr)
{
	unsigned i;

	REG32(csr, DMA_BUS_MODE) |= DMA_BUS_MODE_SFT_RESET;
	for (i = 0u; i < 100000u; i++) {
		if ((REG32(csr, DMA_BUS_MODE) & DMA_BUS_MODE_SFT_RESET) == 0u)
			return 0;
	}
	return -1;
}

/* 写 MAC 地址寄存器 0；GMAC_HI_REG_AE 置位表示该地址槽有效。多播过滤等
 * 仍受 PACKET_FILTER 影响。 */
static void set_mac_addr(uintptr_t csr, const uint8_t *mac)
{
	uint32_t hi;
	uint32_t lo;

	hi = ((uint32_t)mac[5] << 8) | (uint32_t)mac[4];
	hi |= GMAC_HI_REG_AE;
	lo = ((uint32_t)mac[3] << 24) | ((uint32_t)mac[2] << 16) |
	     ((uint32_t)mac[1] << 8) | (uint32_t)mac[0];
	REG32(csr, GMAC_ADDR_HIGH(0)) = hi;
	REG32(csr, GMAC_ADDR_LOW(0)) = lo;
}

/* 从 HW_FEATURE1 读出 IP 综合时的 Tx/Rx MTL FIFO 深度编码，换算成字节。
 * 若读回异常小值（未复位时的垃圾），用下限避免 MTL 配置非法。 */
static unsigned int fifo_from_feat1(uint32_t hw1, unsigned is_tx)
{
	unsigned v;

	if (is_tx)
		v = (unsigned)((hw1 & GMAC_HW_TXFIFOSIZE) >> 6);
	else
		v = (unsigned)(hw1 & GMAC_HW_RXFIFOSIZE);
	return 128u << v;
}

/*
 * MAC + MTL + DMA 通道寄存器初始化（不建环、不启动 ST/SR）。
 * TE/RE 在此末尾保持 0，避免在环地址未就绪时 DMA 乱跑。
 */
void gmac_mac_init(gmac_hal_context_t *hal)
{
	uintptr_t csr;
	uint32_t hw1, mtl, tx_op, rx_op, txc, rxc, dcc;
	unsigned tx_fifo, rx_fifo, tqs, rqs;

	if (!hal) {
		GMAC_BRUP("[EQOS] gmac_mac_init: hal is NULL\n");
		return;
	}

	s_hal_for_newdma = hal;
	csr = hal->csr_base;
	hal->mdio.csr_base = csr;
	/* 仅给 MDIO 分频计算用默认 100MHz；与真实 CSR 时钟不符时 MDC 会过慢/过快，
	 * 需在 eqos_mdio.c 的 CR 选择与 PHY 手册限值内修正。 */
	if (hal->mdio.csr_clk_hz == 0u)
		hal->mdio.csr_clk_hz = 100000000u;

	if (dma_soft_reset(csr) != 0)
		GMAC_BRUP("[EQOS] DMA soft reset: TIMEOUT (DMA_BUS_MODE SFT_RESET stuck), csr=%#lx\n",
			  (unsigned long)csr);
	else
		GMAC_BRUP("[EQOS] DMA soft reset: OK, csr=%#lx\n", (unsigned long)csr);

	/* 声明 AXI 上允许的 burst 长度；FB 等与 fixed burst / 外设能力相关，
	 * 与 Linux 默认组合一致；若 AXI 从机不支持某些长度需按集成裁剪。 */
	REG32(csr, DMA_SYS_BUS_MODE) = DMA_BURST_LEN_DEFAULT | DMA_SYS_BUS_FB;

	/* RAA_SP + SP 调度：多队列场景下的仲裁策略；单队列时仍建议与 stmmac
	 * 常用配置一致，减少与集成假设不一致带来的边界问题。 */
	mtl = REG32(csr, MTL_OPERATION_MODE);
	mtl &= ~(MTL_OPERATION_RAA | MTL_OPERATION_SCHALG_MASK);
	mtl |= MTL_OPERATION_RAA_SP | MTL_OPERATION_SCHALG_SP;
	REG32(csr, MTL_OPERATION_MODE) = mtl;

	/* 全部 Rx 队列映射到 DMA ch0；多队列 SoC 若需 RSS 等要改映射寄存器。 */
	REG32(csr, MTL_RXQ_DMA_MAP0) = 0u;

	hw1 = REG32(csr, GMAC_HW_FEATURE1);
#if !defined(GMAC_EQOS_SILENT)
	GMAC_BRUP("[EQOS] IDs: GMAC_VERSION@0x20=0x%08x GMAC4_VERSION@0x110=0x%08x | feat1=0x%08x feat2=0x%08x\n",
		  REG32(csr, GMAC_VERSION), REG32(csr, GMAC4_VERSION), hw1,
		  REG32(csr, GMAC_HW_FEATURE2));
#endif
	tx_fifo = fifo_from_feat1(hw1, 1);
	rx_fifo = fifo_from_feat1(hw1, 0);
	/* FEATURE 读失败或未复位时编码可能极小，给默认深度避免 TQS/RQS=0 非法。 */
	if (tx_fifo < 512u)
		tx_fifo = 4096u;
	if (rx_fifo < 512u)
		rx_fifo = 4096u;
	tqs = (tx_fifo / 256u) - 1u;
	if (tqs > 31u)
		tqs = 31u;
	rqs = (rx_fifo / 256u) - 1u;
	if (rqs > 31u)
		rqs = 31u;

	tx_op = REG32(csr, MTL_CHAN_BASE_ADDR + 0u * MTL_CHAN_BASE_OFFSET);
	tx_op &= ~(MTL_OP_MODE_TXQEN_MASK | MTL_OP_MODE_TQS_MASK);
	/* TSF/RSF：Store-and-Forward，整帧进入 MTL 再交 DMA，简化时序、避免切包，
	 * 延迟略高于 threshold 模式。TXQEN 打开队列。 */
	tx_op |= MTL_OP_MODE_TSF | MTL_OP_MODE_TXQEN |
		  (tqs << MTL_OP_MODE_TQS_SHIFT);
	REG32(csr, MTL_CHAN_BASE_ADDR + 0u * MTL_CHAN_BASE_OFFSET) = tx_op;

	/* Rx 队列 0 操作寄存器在通道块 +0x30（与 dwmac4 布局一致）。 */
	rx_op = REG32(csr, MTL_CHAN_BASE_ADDR + 0u * MTL_CHAN_BASE_OFFSET +
			     0x30u);
	rx_op &= ~MTL_OP_MODE_RQS_MASK;
	rx_op |= MTL_OP_MODE_RSF | (rqs << MTL_OP_MODE_RQS_SHIFT);
	REG32(csr, MTL_CHAN_BASE_ADDR + 0u * MTL_CHAN_BASE_OFFSET + 0x30u) =
		rx_op;

	/* RBSZ：每个 Rx 描述符 buffer1 最大字节数，须与软件 rx 缓冲一致；
	 * RPBL：Rx DMA 一次突发长度相关位域（与 Linux 使用同一 shift 约定）。 */
	rxc = REG32(csr, DMA_CHAN_RX_CONTROL(EQOS_CHAN));
	rxc &= ~(DMA_RBSZ_MASK | (0x3fu << DMA_BUS_MODE_RPBL_SHIFT));
	rxc |= (EQOS_RX_BUF_SZ << DMA_RBSZ_SHIFT) & DMA_RBSZ_MASK;
	rxc |= (EQOS_PBL << DMA_BUS_MODE_RPBL_SHIFT);
	REG32(csr, DMA_CHAN_RX_CONTROL(EQOS_CHAN)) = rxc;

	/* OSP：单包模式，一描述符一包，与本驱动 TX 填法一致。 */
	txc = REG32(csr, DMA_CHAN_TX_CONTROL(EQOS_CHAN));
	txc &= ~(0x3fu << DMA_BUS_MODE_PBL_SHIFT);
	txc |= DMA_CONTROL_OSP | (EQOS_PBL << DMA_BUS_MODE_PBL_SHIFT);
	REG32(csr, DMA_CHAN_TX_CONTROL(EQOS_CHAN)) = txc;

	dcc = REG32(csr, DMA_CHAN_CONTROL(EQOS_CHAN));
	dcc |= DMA_BUS_MODE_PBL;
	REG32(csr, DMA_CHAN_CONTROL(EQOS_CHAN)) = dcc;

	/* 裸机轮询：关所有 DMA 通道中断，避免未挂 handler 时误触发。 */
	REG32(csr, DMA_CHAN_INTR_ENA(EQOS_CHAN)) = 0u;

	/* RA|PM：混杂/全收类过滤，bring-up 阶段便于环回与抓任意 DA 的帧；
	 * 产品化常改为精确 DA + 广播/多播策略。 */
	REG32(csr, GMAC_PACKET_FILTER) =
		GMAC_PACKET_FILTER_RA | GMAC_PACKET_FILTER_PM;

	{
		static const uint8_t mac[6] = { 0x02, 0x00, 0x00, 0x00, 0x00, 0x01 };

		set_mac_addr(csr, mac);
	}

	{
		uint32_t cfg = REG32(csr, GMAC_CONFIG);

		/* DM：千兆/字节对齐 MII 数据宽度侧；PS 清：非 MII 8bit，与 RGMII
		 * 常用配置一致。TE/RE 在此关闭，等环就绪后在 new_dma 再开。 */
		cfg |= GMAC_CONFIG_DM;
		cfg &= ~GMAC_CONFIG_PS;
		cfg &= ~(GMAC_CONFIG_TE | GMAC_CONFIG_RE);
#if GMAC_EQOS_MAC_LOOPBACK_DEFAULT
		/* LM：MAC 内部环回，PHY 不参与；外发真实网络必须关闭。 */
		cfg |= GMAC_CONFIG_LM;
#else
		cfg &= ~GMAC_CONFIG_LM;
#endif
		REG32(csr, GMAC_CONFIG) = cfg;
	}

#if !defined(GMAC_EQOS_SILENT)
	{
		uint32_t gcf = REG32(csr, GMAC_CONFIG);
		uint32_t dma_st = REG32(csr, DMA_STATUS);

		GMAC_BRUP("[EQOS] MTL: tx_fifo=%u rx_fifo=%u bytes -> TQS=%u RQS=%u | ring_rx_buf=%u PBL=%u\n",
			  tx_fifo, rx_fifo, tqs, rqs, EQOS_RX_BUF_SZ, EQOS_PBL);
		GMAC_BRUP("[EQOS] MAC: addr0 02:00:00:00:00:01 | GMAC_CONFIG=0x%08x TE=%d RE=%d LM=%d DM=%d PS=%d\n",
			  gcf,
			  (gcf & GMAC_CONFIG_TE) ? 1 : 0, (gcf & GMAC_CONFIG_RE) ? 1 : 0,
			  (gcf & GMAC_CONFIG_LM) ? 1 : 0, (gcf & GMAC_CONFIG_DM) ? 1 : 0,
			  (gcf & GMAC_CONFIG_PS) ? 1 : 0);
		GMAC_BRUP("[EQOS] MDIO: csr_clk_hz=%lu (CR 分频请与 eqos_mdio 一致)\n",
			  (unsigned long)hal->mdio.csr_clk_hz);
		GMAC_BRUP("[EQOS] DMA_STATUS=0x%08x (init 阶段，尚未建环)\n", dma_st);
	}
#endif
}

/* 把 Rx 描述符交还硬件：buffer1 地址 + OWN=1。须在 CPU 可见缓冲写回完成
 * 后再置 OWN（含 cache clean）。 */
static void rx_desc_fill(struct gmac_esp_dma_state *d, unsigned idx)
{
	struct eqos_dma_desc *p = &d->rx_ring[idx];
	uint32_t a = dma_phys(d->rx_buf[idx]);

	p->des0 = a;
	p->des1 = 0u;
	p->des2 = 0u;
	p->des3 = RDES3_OWN | RDES3_BUFFER1_VALID_ADDR;
	dmb();
}

/*
 * 建立 Tx/Rx 环、告知 DMA 基址与深度、写 Rx tail 初值、启动 DMA 与 MAC TE/RE。
 * Rx tail（RX_END_ADDR）通常指向「当前软件消费索引」描述符，驱动写回描述符
 * 后推进 tail 作为门铃，通知硬件有新空描述符可用。
 */
void gmac_esp_new_dma(gmac_esp_dma_handle_t *out_handle)
{
	uintptr_t csr;
	unsigned i;
	uint32_t txb, rxb, rtl;

	if (!out_handle) {
		GMAC_BRUP("[EQOS] gmac_esp_new_dma: out_handle is NULL\n");
		return;
	}

	memset(&s_dma, 0, sizeof(s_dma));
	s_dma.hal = s_hal_for_newdma;
	if (!s_dma.hal) {
		GMAC_BRUP("[EQOS] gmac_esp_new_dma: call gmac_mac_init() first (s_hal_for_newdma NULL)\n");
		*out_handle = NULL;
		return;
	}

	csr = s_dma.hal->csr_base;

	for (i = 0u; i < EQOS_RING_N; i++) {
		memset(&s_dma.tx_ring[i], 0, sizeof(s_dma.tx_ring[i]));
		rx_desc_fill(&s_dma, i);
	}

	/* 深度寄存器存 (N-1)，与 Linux 一致。环须连续物理内存：本实现为静态数组，
	 * 若链接脚本不保证连续，需改为整块 dma_alloc。 */
	REG32(csr, DMA_CHAN_TX_RING_LEN(EQOS_CHAN)) = EQOS_RING_N - 1u;
	REG32(csr, DMA_CHAN_RX_RING_LEN(EQOS_CHAN)) = EQOS_RING_N - 1u;

	txb = dma_phys(&s_dma.tx_ring[0]);
	rxb = dma_phys(&s_dma.rx_ring[0]);
	REG32(csr, DMA_CHAN_TX_BASE_ADDR_HI(EQOS_CHAN)) = 0u;
	REG32(csr, DMA_CHAN_TX_BASE_ADDR(EQOS_CHAN)) = txb;
	REG32(csr, DMA_CHAN_RX_BASE_ADDR_HI(EQOS_CHAN)) = 0u;
	REG32(csr, DMA_CHAN_RX_BASE_ADDR(EQOS_CHAN)) = rxb;

	/* Rx tail 初值指向环尾下一槽（此处即「最后一个描述符之后」），与
	 * rx_cons=0 配对；回收描述符后 tail 推进到当前消费索引。 */
	rtl = rxb + (unsigned int)(EQOS_RING_N * sizeof(struct eqos_dma_desc));
	REG32(csr, DMA_CHAN_RX_END_ADDR(EQOS_CHAN)) = rtl;
	/* Tx tail 初值指向环首：尚未提交包时与 stmmac 习惯一致。 */
	REG32(csr, DMA_CHAN_TX_END_ADDR(EQOS_CHAN)) = txb;

	dmb();

	/* 先 ST/SR 再 TE/RE 为常见顺序；若仍无收发，查时钟/复位/环物理地址。 */
	REG32(csr, DMA_CHAN_TX_CONTROL(EQOS_CHAN)) |= DMA_CONTROL_ST;
	REG32(csr, GMAC_CONFIG) |= GMAC_CONFIG_TE;

	REG32(csr, DMA_CHAN_RX_CONTROL(EQOS_CHAN)) |= DMA_CONTROL_SR;
	REG32(csr, GMAC_CONFIG) |= GMAC_CONFIG_RE;

	s_dma.running = 1;
	s_dma.tx_head = 0u;
	s_dma.rx_cons = 0u;
	*out_handle = &s_dma;

	GMAC_BRUP("[EQOS] DMA ch%u: ring_depth=%u desc_size=%zu rx_buf=%u\n",
		  EQOS_CHAN, EQOS_RING_N, sizeof(struct eqos_dma_desc), EQOS_RX_BUF_SZ);
	GMAC_BRUP("[EQOS] DMA ring phys: tx_ring=%#x rx_ring=%#x rx_tail_doorbell=%#x\n",
		  txb, rxb, rtl);
	GMAC_BRUP("[EQOS] DMA regs: BUS_MODE=0x%08x SYS_BUS=0x%08x STATUS=0x%08x\n",
		  REG32(csr, DMA_BUS_MODE), REG32(csr, DMA_SYS_BUS_MODE),
		  REG32(csr, DMA_STATUS));
	GMAC_BRUP("[EQOS] DMA ch%u: TXC=0x%08x RXC=0x%08x | GMAC_CONFIG=0x%08x (TE/RE 应已置位)\n",
		  EQOS_CHAN,
		  REG32(csr, DMA_CHAN_TX_CONTROL(EQOS_CHAN)),
		  REG32(csr, DMA_CHAN_RX_CONTROL(EQOS_CHAN)),
		  REG32(csr, GMAC_CONFIG));
}

/*
 * 单描述符发送：长度受 TDES2/TDES3 域宽与 EQOS_RX_BUF_SZ 限制。
 * 顺序：清上一帧 OWN → memcpy → 写 des0..2 → 写 des3（无 OWN）→ dmb →
 * des3|=OWN → 写 TX_END_ADDR 门铃。若 OWN 长期不清，优先查 TX 缓冲与描述符
 * 是否对 DMA 可见（cache）、门铃地址是否写对通道。
 */
int32_t gmac_esp_dma_transmit_frame(gmac_esp_dma_handle_t dma, uint8_t *buf,
				    uint32_t length, gmac_hal_context_t *hal)
{
	struct gmac_esp_dma_state *d = dma;
	struct eqos_dma_desc *desc;
	uint32_t des3, phys;
	unsigned idx;
	unsigned spin;

	if (!hal || !buf || length == 0u)
		return 0;
	if (!d)
		d = &s_dma;
	if (length > EQOS_RX_BUF_SZ)
		return 0;

	idx = d->tx_head % EQOS_RING_N;
	desc = &d->tx_ring[idx];
	/* OWN=1 仍属硬件：环满应退避或等完成中断/轮询，不可覆盖。 */
	if (desc->des3 & TDES3_OWN)
		return 0;

	memcpy(d->tx_buf[idx], buf, length);
	dmb();

	phys = dma_phys(d->tx_buf[idx]);
	desc->des0 = phys;
	desc->des1 = 0u;
	desc->des2 = length & TDES2_BUFFER1_SIZE_MASK;
	des3 = (length & TDES3_PACKET_SIZE_MASK);
	/* 裸机不_OFFLOAD 校验和：清 insertion 位，避免硬件向帧内写错误层级的和。 */
	des3 &= ~(TDES3_CHECKSUM_INSERTION_MASK);
	des3 |= TDES3_FIRST_DESCRIPTOR | TDES3_LAST_DESCRIPTOR;
	desc->des3 = des3;
	dmb();
	desc->des3 = des3 | TDES3_OWN;
	dmb();

	REG32(hal->csr_base, DMA_CHAN_TX_END_ADDR(EQOS_CHAN)) =
		dma_phys(desc);

	for (spin = 0u; spin < 1000000u; spin++) {
		if ((desc->des3 & TDES3_OWN) == 0u)
			break;
	}
	if (desc->des3 & TDES3_OWN) {
		GMAC_BRUP("[EQOS] TX timeout: slot=%u OWN still 1, TDES3=0x%08x len=%lu bus dead或门铃/Cache?\n",
			  idx, desc->des3, (unsigned long)length);
		return 0;
	}

	d->tx_head = (d->tx_head + 1u) % EQOS_RING_N;
	return (int32_t)length;
}

/*
 * 轮询消费一帧 Rx：仅处理「单缓冲、FIRST+LAST 同描述符」的常见情况。
 * PTP/校验和卸载可能插入 CONTEXT 描述符，此处跳过并回收环槽。
 * done_rx：无论是否解析出「业务 ok」，都 refill 描述符并推进 rx_cons、
 * 写 RX_END_ADDR，否则 DMA 认为无可用缓冲会停收。
 */
bool gmac_receive_frame(gmac_hal_context_t *hal, uint16_t *expected_frame,
			bool check_content_flag, bool retrun_on_every_good_pkt,
			bool timestamp_print)
{
	struct gmac_esp_dma_state *d = &s_dma;
	struct eqos_dma_desc *desc;
	uint32_t r3;
	unsigned idx;
	uint32_t pl;
	gmac_frame_t *pkt;
	bool ok = false;
	static unsigned log_ctx, log_layout, log_pl, log_es, log_eth, log_pay;

	(void)timestamp_print;

	if (!hal || !d->running)
		return false;

	idx = d->rx_cons % EQOS_RING_N;
	desc = &d->rx_ring[idx];
	r3 = desc->des3;
	/* OWN=1：硬件仍占有，无完整帧可读。 */
	if (r3 & RDES3_OWN)
		return false;
	if (r3 & RDES3_CONTEXT_DESCRIPTOR) {
		if (log_ctx < 8u) {
			log_ctx++;
			GMAC_BRUP("[EQOS] RX: context descriptor skip, RDES3=0x%08x (%u/8)\n", r3,
				  log_ctx);
		}
		goto done_rx;
	}
	if (!(r3 & RDES3_FIRST_DESCRIPTOR) || !(r3 & RDES3_LAST_DESCRIPTOR)) {
		if (log_layout < 12u) {
			log_layout++;
			GMAC_BRUP("[EQOS] RX: not single-segment (FIRST/LAST), RDES3=0x%08x (%u/12)\n",
				  r3, log_layout);
		}
		goto done_rx;
	}

	pl = r3 & RDES3_PACKET_SIZE_MASK;
	if (pl < ETH_HEADER_LEN || pl > EQOS_RX_BUF_SZ) {
		if (log_pl < 8u) {
			log_pl++;
			GMAC_BRUP("[EQOS] RX: bad packet len pl=%u (rdes3=0x%08x) (%u/8)\n", pl, r3,
				  log_pl);
		}
		goto done_rx;
	}
	if (r3 & RDES3_ERROR_SUMMARY) {
		if (log_es < 16u) {
			log_es++;
			GMAC_BRUP("[EQOS] RX: ERROR_SUMMARY pl=%u RDES3=0x%08x (%u/16)\n", pl, r3,
				  log_es);
		}
		goto done_rx;
	}

	pkt = (gmac_frame_t *)d->rx_buf[idx];
	if (pkt->proto == GMAC_NTOHS(GMAC_MY_NORMAL_PKT_TYPE)) {
		if (expected_frame)
			(*expected_frame)++;
		if (check_content_flag) {
			unsigned j;

			for (j = 0u; j + ETH_HEADER_LEN < pl; j++) {
				if (pkt->data[j] != (uint8_t)(j & 0xff)) {
					if (log_pay < 8u) {
						log_pay++;
						GMAC_BRUP("[EQOS] RX: payload mismatch at j=%u got=0x%02x (%u/8)\n",
							  j, pkt->data[j], log_pay);
					}
					goto done_rx;
				}
			}
			if (retrun_on_every_good_pkt)
				ok = true;
		} else if (retrun_on_every_good_pkt) {
			ok = true;
		}
	} else if (pkt->proto == GMAC_NTOHS(GMAC_MY_STOP_PKT_TYPE)) {
		ok = true;
	} else {
		if (log_eth < 16u) {
			log_eth++;
			GMAC_BRUP("[EQOS] RX: ethertype 0x%04x len=%u (expect 0x%04x/0x%04x) (%u/16)\n",
				  GMAC_NTOHS(pkt->proto), pl,
				  GMAC_MY_NORMAL_PKT_TYPE, GMAC_MY_STOP_PKT_TYPE, log_eth);
		}
		ok = false;
		goto done_rx;
	}

done_rx:
	rx_desc_fill(d, idx);
	d->rx_cons = (d->rx_cons + 1u) % EQOS_RING_N;
	REG32(hal->csr_base, DMA_CHAN_RX_END_ADDR(EQOS_CHAN)) =
		dma_phys(&d->rx_ring[d->rx_cons]);
	return ok;
}
