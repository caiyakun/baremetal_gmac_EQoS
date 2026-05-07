/*
 * Synopsys EQoS 5.40a / DWMAC4 单通道裸机 HAL
 *
 * 参考：
 *   - Linux drivers/net/ethernet/stmicro/stmmac（dwmac4_*、DMA 通道寄存器布局）
 *   - P5 .../test_gmac.c 中 test_mac_near_end_loopback_force_link（PHY + MAC LM + 发包校验）
 *
 * 日志：统一使用 GMAC_PRINTF（默认 printf）；可在包含 gmac_eqos_hal.h 前
 *       #define GMAC_PRINTF(...)  your_uart_printf(__VA_ARGS__) 重定向到串口。
 *
 * 数据路径：静态 TX/RX 描述符环 + 包缓冲；发送/接收前后对描述符与缓冲区做
 *           DMA_CACHE_WB / DMA_CACHE_INVALIDATE（weak 空实现适用于无 D-cache 环境）。
 */
#include "gmac_eqos_hal.h"
#include "eqos_regs.h"
#include "eqos_desc.h"

#include <string.h>

#define EQOS_DMA_DESC_SZ	sizeof(struct eqos_dma_desc)

#ifndef EQOS_VIRT_TO_PHYS
#define EQOS_VIRT_TO_PHYS(p) ((uint32_t)(uintptr_t)(p))
#endif

#ifndef EQOS_MEM_BARRIER
#define EQOS_MEM_BARRIER() __sync_synchronize()
#endif

#ifndef GMAC_PRINTF
#include <stdio.h>
#define GMAC_PRINTF(...) printf(__VA_ARGS__)
#endif

#ifndef MAC_TEST_SRC_ADDR
#define MAC_TEST_SRC_ADDR 0x02, 0x00, 0x00, 0x00, 0x00, 0x01
#endif

#define EQOS_TX_RING	4U
#define EQOS_RX_RING	8U
#define EQOS_BUF_SZ	2048U

#define PBL_VAL		8U

static const char *eqos_speed_str(gmac_speed_t s)
{
	switch (s) {
	case GMAC_SPEED_10M:
		return "10M";
	case GMAC_SPEED_100M:
		return "100M";
	case GMAC_SPEED_1000M:
	default:
		return "1000M";
	}
}

typedef struct {
	struct eqos_dma_desc *tx_desc;
	struct eqos_dma_desc *rx_desc;
	uint8_t (*tx_buf)[EQOS_BUF_SZ];
	uint8_t (*rx_buf)[EQOS_BUF_SZ];
	unsigned tx_desc_num;
	unsigned rx_desc_num;
	unsigned buf_size;
} gmac_eqos_dma_t;

/*
 * 裸机版 DMA 内存池：
 * - s_txd/s_rxd：真实 DMA 描述符 ring 存储。
 * - s_txb/s_rxb：真实 TX/RX frame buffer 存储。
 *
 * 后面会在 gmac_eqos_new_dma() 中把这些静态内存绑定到 s_dma->tx_buf/rx_buf，
 * 这样阅读结构和 P5 的 gmac_esp_dma->tx_buf[i] = g_tx_buffers[i] 更接近。
 */
static struct eqos_dma_desc s_txd[EQOS_TX_RING] __attribute__((aligned(64)));
static struct eqos_dma_desc s_rxd[EQOS_RX_RING] __attribute__((aligned(64)));
static uint8_t s_txb[EQOS_TX_RING][EQOS_BUF_SZ] __attribute__((aligned(32)));
static uint8_t s_rxb[EQOS_RX_RING][EQOS_BUF_SZ] __attribute__((aligned(32)));

static gmac_eqos_dma_t s_eqos_dma;
static gmac_eqos_dma_t *s_dma;
static gmac_hal_context_t *s_hal;
static unsigned s_csr_hz;
static uint32_t s_rx_scan;
static uintptr_t s_test_csr;
static unsigned s_test_csr_hz;

static inline uint32_t reg_rd(gmac_hal_context_t *h, uint32_t off)
{
	return gmac_io_read32(h->csr_base + (uintptr_t)off);
}

static inline void reg_wr(gmac_hal_context_t *h, uint32_t off, uint32_t v)
{
	gmac_io_write32(h->csr_base + (uintptr_t)off, v);
}

void gmac_eqos_set_csr_clock_hz(unsigned hz)
{
	s_csr_hz = hz;
}

static int dma_soft_reset(gmac_hal_context_t *h)
{
	uint32_t v = reg_rd(h, DMA_BUS_MODE);

	/*
	 * DMA_BUS_MODE.SFT_RESET:
	 * 对 DMA common + channel 相关寄存器做软复位。复位完成后硬件自动清零。
	 * 这一步等价于 P5/驱动初始化最前面的 DMA reset，避免上一次测试残留 ST/SR、
	 * tail pointer、中断状态等影响本次配置。
	 */
	reg_wr(h, DMA_BUS_MODE, v | DMA_BUS_MODE_SFT_RESET);
	for (unsigned i = 0; i < 100000U; i++) {
		v = reg_rd(h, DMA_BUS_MODE);
		if (!(v & DMA_BUS_MODE_SFT_RESET))
			return 0;
	}
	return -1;
}

static void tx_desc_prepare(struct eqos_dma_desc *d, uint32_t buf_phys,
			     unsigned len, unsigned tot_len)
{
	memset(d, 0, sizeof(*d));
	d->des0 = buf_phys;
	d->des1 = 0;
	d->des2 = len & TDES2_BUFFER1_SIZE_MASK;
	d->des3 = (tot_len & TDES3_PACKET_SIZE_MASK) | TDES3_FIRST_DESCRIPTOR |
		  TDES3_LAST_DESCRIPTOR;
	EQOS_MEM_BARRIER();
	d->des3 |= TDES3_OWN;
	EQOS_MEM_BARRIER();
	DMA_CACHE_WB(d, EQOS_DMA_DESC_SZ);
}

static void rx_desc_reset_owned(struct eqos_dma_desc *d, uint32_t buf_phys)
{
	/*
	 * 这是 RX 描述符的“初始化/回收”函数：
	 * 软件把一个 RX buffer 地址写进描述符，然后把 OWN 置 1 交给 DMA。
	 * 之后 DMA 收到帧时会把帧数据写到 des0 指向的 buffer，并在完成后清 OWN。
	 */
	memset(d, 0, sizeof(*d));

	/*
	 * RDES0/RDES1：
	 * 在 DWMAC4 normal RX descriptor 的 read-format 中，des0/des1 用作 buffer 地址。
	 * 当前裸机只使用 buffer1，并且默认地址在 32-bit 空间内，所以：
	 *   des0 = buffer 低 32 位物理地址
	 *   des1 = buffer 高 32 位，固定写 0
	 */
	d->des0 = buf_phys;
	d->des1 = 0; //cai note:如果是64bit 空间地址，并且高32bit有值的话，需要填写此字段

	/*
	 * RDES2：
	 * RX read-format 下可承载 buffer2 地址或扩展控制信息。本 HAL 只用单 buffer1，
	 * 因此清 0，避免上一次 DMA write-back 状态残留。
	 */
	d->des2 = 0;

	/*
	 * RDES3 read-format 控制位：
	 * - RDES3_BUFFER1_VALID_ADDR：告诉 DMA des0/des1 中的 buffer1 地址有效。
	 * - RDES3_INT_ON_COMPLETION_EN：DMA 写完这个描述符后可产生 RX complete 中断。
	 *   即使当前测试以轮询为主，也保留该位，便于通过 DMA_CHAN_STATUS.RI 观察收包。
	 */
	d->des3 = RDES3_BUFFER1_VALID_ADDR | RDES3_INT_ON_COMPLETION_EN;

	/*
	 * 内存屏障：
	 * 必须保证 des0/des1/des2/des3 的地址与控制位先写到内存，再置 OWN。
	 * 否则 DMA 可能先看到 OWN=1，却还没看到正确的 buffer 地址。
	 */
	EQOS_MEM_BARRIER();

	/*
	 * RDES3.OWN：
	 * OWN=1 表示描述符归 DMA 所有，软件不能再修改；DMA 收到帧并完成 write-back 后
	 * 会清 OWN，软件在 gmac_receive_frame() 中看到 OWN=0 才能读取帧内容和状态。
	 */
	d->des3 |= RDES3_OWN;

	/* 再次屏障，保证 OWN 置位不会被重排到 cache write-back 之后。 */
	EQOS_MEM_BARRIER();

	/*
	 * Cache 维护：
	 * 将描述符内容写回内存，确保非 cache-coherent DMA 能看到刚才写入的地址/控制/OWN。
	 * 无 D-cache 环境下 DMA_CACHE_WB 是 weak 空实现，不影响功能。
	 */
	DMA_CACHE_WB(d, EQOS_DMA_DESC_SZ);
}

static gmac_eqos_dma_t *gmac_eqos_new_dma(void)
{
	/*
	 * 对齐 P5 gmac_esp_new_dma() 的语义：
	 * P5 是把 g_descriptors / g_tx_buffers / g_rx_buffers 绑定到
	 * gmac_esp_dma->descriptors / tx_buf[] / rx_buf[]。
	 *
	 * 这里不做动态分配，直接把本文件的静态内存池绑定到 s_eqos_dma，
	 * 后续所有 TX/RX 路径都通过 s_dma->tx_buf / s_dma->rx_buf 访问。
	 */
	s_eqos_dma.tx_desc = s_txd;
	s_eqos_dma.rx_desc = s_rxd;
	s_eqos_dma.tx_buf = s_txb;
	s_eqos_dma.rx_buf = s_rxb;
	s_eqos_dma.tx_desc_num = EQOS_TX_RING;
	s_eqos_dma.rx_desc_num = EQOS_RX_RING;
	s_eqos_dma.buf_size = EQOS_BUF_SZ;

	GMAC_PRINTF("EQoS DMA: new_dma bind tx_desc=%p rx_desc=%p tx_buf=%p rx_buf=%p\n",
		    (void *)s_eqos_dma.tx_desc, (void *)s_eqos_dma.rx_desc,
		    (void *)s_eqos_dma.tx_buf, (void *)s_eqos_dma.rx_buf);
	return &s_eqos_dma;
}

static void dma_prog_rx_tail_last(gmac_hal_context_t *h, unsigned last_idx)
{
	uint32_t a = EQOS_VIRT_TO_PHYS(&s_dma->rx_desc[last_idx]);

	/*
	 * DMA_CHAN_RX_END_ADDR:
	 * RX tail pointer。DWMAC4 ring 模式下，软件把可用 RX 描述符重新交给 DMA 后，
	 * 需要写 tail pointer 通知 DMA 描述符窗口已推进；初始化时指向最后一项表示整环可用。
	 */
	reg_wr(h, DMA_CHAN_RX_END_ADDR, a);
}

static void dma_prog_tx_tail(gmac_hal_context_t *h, unsigned idx)
{
	uint32_t a = EQOS_VIRT_TO_PHYS(&s_dma->tx_desc[idx]);

	/*
	 * DMA_CHAN_TX_END_ADDR:
	 * TX tail pointer。软件填好 TX 描述符并置 OWN 后，写入对应描述符地址；
	 * DMA 看到 tail 更新后开始取描述符并搬运待发送帧。
	 */
	reg_wr(h, DMA_CHAN_TX_END_ADDR, a);
}

static void gmac_hal_init_mtl_default(gmac_hal_context_t *h)
{
	uint32_t v;

	GMAC_PRINTF("EQoS HAL: gmac_hal_init_mtl_default — 配置 MTL queue0 与 RX queue 映射\n");

	/*
	 * MTL_CHAN_TX_OP_MODE:
	 * - TXQEN：使能 queue0 参与调度，否则 MAC TX 即使打开也没有队列输出。
	 * - TSF 清零：关闭 TX Store-and-Forward，TX FIFO 不再等待整帧进入后才发送。
	 * - TTC=64B：关闭 TSF 后必须选择 threshold；这里对齐 P5 的
	 *   EMAC_LL_TRANSMIT_THRESHOLD_CONTROL_64，FIFO 累积到 64B 左右即可开始向 MAC 发送。
	 *
	 * 注意：MTL_OP_MODE_DIS_TCP_EF 是 RX op-mode 位，不能写到 TX op-mode；
	 * TX op-mode bit[6:4] 是 TTC threshold 字段。
	 */
	v = reg_rd(h, MTL_CHAN_TX_OP_MODE);
	v &= ~MTL_OP_MODE_TXQEN_MASK;
	v &= ~MTL_OP_MODE_TSF;
	v &= ~MTL_OP_MODE_TTC_MASK;
	v |= MTL_OP_MODE_TXQEN | MTL_OP_MODE_TTC_64;
	reg_wr(h, MTL_CHAN_TX_OP_MODE, v);

	/*
	 * MTL_CHAN_RX_OP_MODE:
	 * - RSF 清零：关闭 RX Store-and-Forward，RX FIFO 不再等待完整帧后才交给 DMA。
	 * - RTC=64B：关闭 RSF 后必须选择 threshold；这里对齐 P5 的
	 *   EMAC_LL_RECEIVE_THRESHOLD_CONTROL_64，FIFO 到 64B 左右即可触发 DMA 写内存。
	 * - DIS_TCP_EF：过滤 checksum error frame，避免错误帧进入 DMA/RX buffer。
	 */
	v = reg_rd(h, MTL_CHAN_RX_OP_MODE);
	v &= ~MTL_OP_MODE_RSF;
	v &= ~MTL_OP_MODE_RTC_MASK;
	v |= MTL_OP_MODE_RTC_64 | MTL_OP_MODE_DIS_TCP_EF;
	reg_wr(h, MTL_CHAN_RX_OP_MODE, v);

	/*
	 * MTL_RXQ_DMA_MAP0:
	 * 把 RX queue0 映射到 DMA channel0。本裸机 HAL 只启用单队列/单通道，
	 * 所以 queue0 -> channel0 是唯一有效路径。
	 */
	v = reg_rd(h, MTL_RXQ_DMA_MAP0);
	v &= ~MTL_RXQ_DMA_QXMDMACH_MASK(0);
	v |= MTL_RXQ_DMA_QXMDMACH(0, 0);
	reg_wr(h, MTL_RXQ_DMA_MAP0, v);

	/*
	 * GMAC_RXQ_CTRL0:
	 * 使能 MAC RX queue0。这里选择 DCB queue enable，与 DWMAC4/stmmac 常见
	 * 单队列配置一致；不使能时 RE 打开也可能没有帧进入 MTL/DMA。
	 */
	v = reg_rd(h, GMAC_RXQ_CTRL0);
	v &= GMAC_RX_QUEUE_CLEAR(0);
	v |= GMAC_RX_DCB_QUEUE_ENABLE(0);
	reg_wr(h, GMAC_RXQ_CTRL0, v);
}

static void gmac_hal_set_address(gmac_hal_context_t *h, const uint8_t *mac)
{
	uint32_t hi = ((uint32_t)mac[5] << 8) | mac[4];

	/*
	 * GMAC_ADDR_HIGH(0):
	 * - 写 MAC 地址高 16 位。
	 * - AE(Address Enable) 置位后地址槽 0 生效，用于 DA 过滤与本机地址匹配。
	 * - DCS 选择 DMA channel，本单通道场景固定为 channel0。
	 */
	hi |= (0U << GMAC_HI_DCS_SHIFT) & GMAC_HI_DCS_MASK;
	reg_wr(h, GMAC_ADDR_HIGH(0), hi | GMAC_HI_REG_AE);

	/*
	 * GMAC_ADDR_LOW(0):
	 * 写 MAC 地址低 32 位。寄存器字节顺序按 Synopsys 地址寄存器格式拼接。
	 */
	reg_wr(h, GMAC_ADDR_LOW(0),
	       ((uint32_t)mac[3] << 24) | ((uint32_t)mac[2] << 16) |
		       ((uint32_t)mac[1] << 8) | mac[0]);
	memcpy(h->mac_addr, mac, 6);
	GMAC_PRINTF("EQoS HAL: MAC 地址已设置为 %02x:%02x:%02x:%02x:%02x:%02x\n",
		    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void gmac_hal_init_mac_default(gmac_hal_context_t *hal)
{
	uint32_t v;

	GMAC_PRINTF("EQoS HAL: gmac_hal_init_mac_default — 配置 GMAC_CONFIG 基础位\n");

	/*
	 * GMAC_CONFIG:
	 * - JD/Jabber Disable：关闭 jabber 限制，调试大帧/异常帧时不被 MAC 提前截断。
	 * - JE/Jumbo Enable：允许 jumbo 相关路径；当前测试仍发送 <=1514B，但保持 P5 default 的宽松配置。
	 * - BE/Frame Burst Enable：使能 burst frame 发送能力。
	 * - DCRS：半双工下禁止 CRS 检查，near-end loopback 调试减少外部载波依赖。
	 * - IPC：打开 RX IP checksum offload 检查，和 P5 ETH_CHECKSUM_HW 同类。
	 * - 清 TE/RE：初始化阶段不立即打开收发，等 gmac_mac_start() 统一启动。
	 * - 清 LM：默认不环回，near-end loopback 由 gmac_mac_near_loopback_prepare() 单独置位。
	 * - 清 PS/FES：默认回到 1000M 编码；实际速率随后由 gmac_mac_set_speed() 按 PHY force 值改写。
	 */
	v = reg_rd(hal, GMAC_CONFIG);
	v |= GMAC_CONFIG_JD | GMAC_CONFIG_JE | GMAC_CONFIG_BE |
	     GMAC_CONFIG_DCRS | GMAC_CONFIG_IPC;
	v &= ~(GMAC_CONFIG_TE | GMAC_CONFIG_RE | GMAC_CONFIG_LM |
	       GMAC_CONFIG_PS | GMAC_CONFIG_FES);
	reg_wr(hal, GMAC_CONFIG, v);

	if (s_csr_hz >= 1000000U) {
		/*
		 * GMAC4_MAC_ONEUS_TIC_COUNTER:
		 * 写入 “CSR 时钟周期数 - 1”，让 MAC 内部 1us tick 与实际 CSR clock 对齐。
		 * PTP/部分超时计数会依赖这个值；不知道 CSR 时钟时跳过，避免写错。
		 */
		reg_wr(hal, GMAC4_MAC_ONEUS_TIC_COUNTER,
		       (s_csr_hz / 1000000U) - 1U);
	}

	/*
	 * GMAC_PHYIF_CONTROL_STATUS:
	 * LUD(Link Up/Down) 置位用于通知 MAC 侧链路状态发生更新。
	 * 在 FPGA RGMII + MAC near-end loopback 场景下，即使不依赖真实线缆链路，
	 * 部分集成仍要求该位被置过，MAC/MTL 才按 link-up 后状态工作。
	 */
	v = reg_rd(hal, GMAC_PHYIF_CONTROL_STATUS);
	v |= GMAC_PHYIF_CTRLSTATUS_LUD;
	reg_wr(hal, GMAC_PHYIF_CONTROL_STATUS, v);
}

static void gmac_hal_init_dma_default(gmac_hal_context_t *hal, unsigned pbl)
{
	uint32_t v;

	GMAC_PRINTF("EQoS HAL: gmac_hal_init_dma_default — 配置 DMA bus/channel，PBL=%u\n",
		    pbl);

	if (dma_soft_reset(hal) != 0)
		GMAC_PRINTF("EQoS HAL: 错误 — DMA_BUS_MODE 软复位超时（DMA 时钟或总线 hang？）\n");
	else
		GMAC_PRINTF("EQoS HAL: DMA 软复位完成\n");

	/*
	 * DMA_SYS_BUS_MODE:
	 * - AAL(Address-Aligned Beats)：AXI/AHB burst 按地址边界对齐，提高互联兼容性。
	 * - FB(Fixed Burst)：使用固定长度 burst，行为更接近 P5 的 fixed/mixed burst 默认路径。
	 */
	v = reg_rd(hal, DMA_SYS_BUS_MODE);
	v |= DMA_SYS_BUS_AAL | DMA_SYS_BUS_FB;
	reg_wr(hal, DMA_SYS_BUS_MODE, v);

	/*
	 * DMA_BUS_MODE:
	 * - DCHE(Descriptor Cache Enable) 清零：本裸机代码手动维护 cache，
	 *   不启用 DMA 内部 descriptor cache，避免和显式 cache flush/invalidate 互相干扰。
	 */
	v = reg_rd(hal, DMA_BUS_MODE);
	v &= ~DMA_BUS_MODE_DCHE;
	reg_wr(hal, DMA_BUS_MODE, v);

	/*
	 * DMA_CHAN_CONTROL:
	 * - PBLX8 清零：TX/RX PBL 字段按寄存器原始单位解释，不乘以 8。
	 *   这样下面写 PBL_VAL=8 时含义明确，和 P5 dma_burst_len 的“直接设置 burst len”对应。
	 */
	v = reg_rd(hal, DMA_CHAN_CONTROL);
	v &= ~DMA_CHAN_CTRL_PBLX8;
	reg_wr(hal, DMA_CHAN_CONTROL, v);

	/*
	 * DMA_CHAN_INTR_ENA:
	 * 打开 normal/abnormal summary、fatal bus error、RX process stopped、
	 * RX buffer unavailable、RX/TX complete 等基础中断位。当前测试轮询为主，
	 * 但打开这些位便于 gmac_get_receive_finish_int_flag() 清 RI，也方便接串口调试。
	 */
	reg_wr(hal, DMA_CHAN_INTR_ENA, DMA_CHAN_INTR_DEFAULT_MASK_4_10);

	/*
	 * DMA_CHAN_RX_CONTROL:
	 * - RXPBL：RX DMA burst length，设为 pbl。
	 * - RBSZ：RX buffer size。DWMAC4 该字段单位由硬件定义为寄存器编码，
	 *   这里沿用 stmmac/dwmac4 路径写 (EQOS_BUF_SZ << 1)，对应 2048B 缓冲。
	 * - 不置 SR：初始化只配参数，真正启动由 gmac_mac_start() 置 SR。
	 */
	v = reg_rd(hal, DMA_CHAN_RX_CONTROL);
	v &= ~DMA_CHAN_RX_CTRL_RXPBL_MASK;
	v |= (pbl << 16) & DMA_CHAN_RX_CTRL_RXPBL_MASK;
	v &= ~DMA_RBSZ_MASK;
	v |= (EQOS_BUF_SZ << 1) & DMA_RBSZ_MASK;
	reg_wr(hal, DMA_CHAN_RX_CONTROL, v);

	/*
	 * DMA_CHAN_TX_CONTROL:
	 * - TXPBL：TX DMA burst length，设为 pbl。
	 * - OSP(Operate on Second Packet)：DMA 可在第一帧状态返回前处理第二帧，
	 *   对齐 P5 opt_second_frame_enable，提高连续发包吞吐。
	 * - TSE 清零：默认不启用 TCP segmentation offload，本测试发送普通以太帧。
	 * - 不置 ST：真正启动由 gmac_mac_start() 置 ST。
	 */
	v = reg_rd(hal, DMA_CHAN_TX_CONTROL);
	v &= ~DMA_CHAN_TX_CTRL_TXPBL_MASK;
	v |= (pbl << 16) & DMA_CHAN_TX_CTRL_TXPBL_MASK;
	v |= DMA_CONTROL_OSP;
	v &= ~DMA_CONTROL_TSE;
	reg_wr(hal, DMA_CHAN_TX_CONTROL, v);
}

static void gmac_hal_init_desc_ring(gmac_hal_context_t *hal)
{
	GMAC_PRINTF("EQoS HAL: gmac_hal_init_desc_ring — 静态描述符/缓冲区绑定到 DMA\n");

	for (unsigned i = 0; i < s_dma->tx_desc_num; i++) {
		memset(&s_dma->tx_desc[i], 0, sizeof(s_dma->tx_desc[i]));
		/* CPU 刚清零 TX 描述符，写回内存后 DMA 才能看到最新的空闲状态。 */
		DMA_CACHE_WB(&s_dma->tx_desc[i], EQOS_DMA_DESC_SZ);
	}
	for (unsigned i = 0; i < s_dma->rx_desc_num; i++)
		rx_desc_reset_owned(&s_dma->rx_desc[i],
				    EQOS_VIRT_TO_PHYS(s_dma->rx_buf[i]));

	/*
	 * DMA_CHAN_RX_BASE_ADDR_HI / DMA_CHAN_RX_BASE_ADDR:
	 * 写 RX 描述符 ring 的 64 位基地址。当前裸机默认 32-bit 地址空间，
	 * 高 32 位写 0，低 32 位写 s_dma->rx_desc 的物理地址。
	 */
	reg_wr(hal, DMA_CHAN_RX_BASE_ADDR_HI, 0);
	reg_wr(hal, DMA_CHAN_RX_BASE_ADDR, EQOS_VIRT_TO_PHYS(s_dma->rx_desc));

	/*
	 * DMA_CHAN_TX_BASE_ADDR_HI / DMA_CHAN_TX_BASE_ADDR:
	 * 写 TX 描述符 ring 的 64 位基地址。与 RX 一样，高位固定 0。
	 */
	reg_wr(hal, DMA_CHAN_TX_BASE_ADDR_HI, 0);
	reg_wr(hal, DMA_CHAN_TX_BASE_ADDR, EQOS_VIRT_TO_PHYS(s_dma->tx_desc));

	/*
	 * DMA_CHAN_RX_RING_LEN / DMA_CHAN_TX_RING_LEN:
	 * DWMAC4 ring length 寄存器写 “描述符数量 - 1”。例如 8 个 RX 描述符写 7。
	 */
	reg_wr(hal, DMA_CHAN_RX_RING_LEN, s_dma->rx_desc_num - 1U);
	reg_wr(hal, DMA_CHAN_TX_RING_LEN, s_dma->tx_desc_num - 1U);

	/*
	 * DMA_CHAN_RX_END_ADDR:
	 * RX tail pointer 指向 ring 最后一项，告诉 DMA 当前可用 RX 描述符窗口覆盖整个 ring。
	 */
	dma_prog_rx_tail_last(hal, s_dma->rx_desc_num - 1U);

	/*
	 * DMA_CHAN_TX_END_ADDR:
	 * TX tail pointer 初始化到第 0 项。后续 gmac_mac_transmit() 准备某个 TX 描述符后，
	 * 会再次写 tail pointer 触发 DMA 拉取该描述符。
	 */
	reg_wr(hal, DMA_CHAN_TX_END_ADDR, EQOS_VIRT_TO_PHYS(&s_dma->tx_desc[0]));
}

static void gmac_eqos_dma_reset(gmac_hal_context_t *hal)
{
	/*
	 * 对齐 P5 test_gmac.c 的 gmac_esp_dma_reset() 命名和职责：
	 * - 重置/初始化 RX 描述符，并把 RX buffer 交给 DMA（OWN=DMA）。
	 * - 重置/初始化 TX 描述符，让 TX 描述符保持 CPU 所有（OWN=CPU/0）。
	 * - 把 RX/TX descriptor ring base、ring length、tail pointer 写入 DMA channel 寄存器。
	 *
	 * 本实现内部使用 DWMAC4 ring mode，所以没有 P5 chained descriptor 里的
	 * Buffer2NextDescAddr 链表字段；ring 的范围由 DMA_CHAN_*_RING_LEN 和 tail pointer 表示。
	 */
	GMAC_PRINTF("EQoS DMA: gmac_eqos_dma_reset — 对齐 P5 gmac_esp_dma_reset 流程\n");
	gmac_hal_init_desc_ring(hal);
}

void gmac_mac_init(gmac_hal_context_t *hal)
{
	const uint8_t defmac[] = { MAC_TEST_SRC_ADDR };

	s_hal = hal;
	s_dma = gmac_eqos_new_dma();
	s_rx_scan = 0;

	GMAC_PRINTF("EQoS HAL: gmac_mac_init — csr=%p TX环=%u RX环=%u 帧缓冲=%u 字节\n",
		    (void *)hal->csr_base, EQOS_TX_RING, EQOS_RX_RING, EQOS_BUF_SZ);

	/* 对齐 P5 gmac_mac_init 主流程：MAC default -> DMA default -> MAC address -> DMA ring。 */
	gmac_hal_init_mac_default(hal);
	gmac_hal_init_dma_default(hal, PBL_VAL);
	gmac_hal_set_address(hal, defmac);
	gmac_eqos_dma_reset(hal);
	gmac_hal_init_mtl_default(hal);

	GMAC_PRINTF("EQoS HAL: gmac_mac_init 完成 — GMAC4_VERSION=0x%08x，已置 RGMII LUD、"
		    "RX tail 指向末项\n",
		    (unsigned)reg_rd(hal, GMAC4_VERSION));
}

/*
 * MAC 近端环回：GMAC_CONFIG.LM + 混杂模式，使本机发出的帧经 MAC 内部环回到 RX DMA。
 * 与 PHY 侧环回独立；RGMII 场景下仍建议 PHY 已 force link（见 test 中 phy_extra）。
 */
void gmac_mac_near_loopback_prepare(gmac_hal_context_t *hal)
{
	uint32_t v;

	GMAC_PRINTF("EQoS HAL: gmac_mac_near_loopback_prepare — 使能 LM 与混杂过滤\n");

	/*
	 * GMAC_PACKET_FILTER:
	 * - PR(Promiscuous Mode)：近端环回调试时接收所有目的地址，避免 DA 过滤影响结果。
	 * - RA(Receive All)：把未通过地址过滤的帧也交给应用/DMA，便于观测异常帧。
	 * - PM 清零：不额外打开 “pass all multicast”，near-end loopback 用 PR/RA 已足够。
	 */
	v = reg_rd(hal, GMAC_PACKET_FILTER);
	v &= ~(GMAC_PACKET_FILTER_PR | GMAC_PACKET_FILTER_PM |
	       GMAC_PACKET_FILTER_RA);
	v |= GMAC_PACKET_FILTER_PR | GMAC_PACKET_FILTER_RA;
	reg_wr(hal, GMAC_PACKET_FILTER, v);

	/*
	 * GMAC_CONFIG.LM:
	 * 打开 MAC 内部 near-end loopback。此时 TX 出来的帧不依赖外部 RGMII 线缆，
	 * 直接回到 MAC RX/MTL/DMA，用于验证 MAC/DMA/descriptor/cache 路径。
	 */
	v = reg_rd(hal, GMAC_CONFIG);
	v |= GMAC_CONFIG_LM;
	reg_wr(hal, GMAC_CONFIG, v);
	GMAC_PRINTF("EQoS HAL: GMAC_CONFIG.LM 已置位\n");
}

void gmac_mac_set_speed(gmac_hal_context_t *hal, gmac_speed_t speed)
{
	uint32_t v = reg_rd(hal, GMAC_CONFIG);

	GMAC_PRINTF("EQoS HAL: gmac_mac_set_speed — %s（GMAC_CONFIG PS/FES）\n",
		    eqos_speed_str(speed));
	/*
	 * GMAC_CONFIG.PS/FES:
	 * - 1000M：PS=0、FES=0。
	 * - 100M ：PS=1、FES=1。
	 * - 10M  ：PS=1、FES=0。
	 * 该设置必须和 PHY force speed 或自协商结果保持一致，否则 MAC 时钟/编码速率不匹配。
	 */
	v &= ~(GMAC_CONFIG_PS | GMAC_CONFIG_FES);
	switch (speed) {
	case GMAC_SPEED_10M:
		v |= GMAC_CONFIG_PS;
		break;
	case GMAC_SPEED_100M:
		v |= GMAC_CONFIG_PS | GMAC_CONFIG_FES;
		break;
	case GMAC_SPEED_1000M:
	default:
		break;
	}
	reg_wr(hal, GMAC_CONFIG, v);
}

void gmac_mac_set_duplex(gmac_hal_context_t *hal, gmac_duplex_t duplex)
{
	uint32_t v = reg_rd(hal, GMAC_CONFIG);

	GMAC_PRINTF("EQoS HAL: gmac_mac_set_duplex — %s\n",
		    duplex == GMAC_DUPLEX_FULL ? "全双工" : "半双工");
	/*
	 * GMAC_CONFIG.DM:
	 * 置位为 full-duplex，清零为 half-duplex。near-end loopback force link 测试
	 * 使用全双工；真实链路时应和 PHY 协商出的 duplex 对齐。
	 */
	if (duplex == GMAC_DUPLEX_FULL)
		v |= GMAC_CONFIG_DM;
	else
		v &= ~GMAC_CONFIG_DM;
	reg_wr(hal, GMAC_CONFIG, v);
}

void gmac_mac_start(gmac_hal_context_t *hal)
{
	uint32_t v;

	GMAC_PRINTF("EQoS HAL: gmac_mac_start — 使能 DMA RX/TX 与 MAC TE/RE\n");
	/*
	 * DMA_CHAN_RX_CONTROL.SR:
	 * 启动 RX DMA state machine。此位必须在 RX 描述符 ring/tail pointer 已配置后再置位。
	 */
	v = reg_rd(hal, DMA_CHAN_RX_CONTROL);
	v |= DMA_CONTROL_SR;
	reg_wr(hal, DMA_CHAN_RX_CONTROL, v);

	/*
	 * DMA_CHAN_TX_CONTROL.ST:
	 * 启动 TX DMA state machine。描述符未准备好时启动也安全，真正发包由 TX tail 更新触发。
	 */
	v = reg_rd(hal, DMA_CHAN_TX_CONTROL);
	v |= DMA_CONTROL_ST;
	reg_wr(hal, DMA_CHAN_TX_CONTROL, v);

	/*
	 * GMAC_CONFIG.RE/TE:
	 * 打开 MAC 接收/发送状态机。P5 参考中也先启动 DMA，再打开 MAC TE/RE，
	 * 避免 MAC 已开始收发但 DMA 尚未接管描述符造成丢帧。
	 */
	v = reg_rd(hal, GMAC_CONFIG);
	v |= GMAC_CONFIG_RE | GMAC_CONFIG_TE;
	reg_wr(hal, GMAC_CONFIG, v);
}

int gmac_mac_stop(gmac_hal_context_t *hal)
{
	uint32_t v;

	GMAC_PRINTF("EQoS HAL: gmac_mac_stop — 关闭 TE/RE 与 DMA 通道\n");
	/*
	 * DMA_CHAN_TX_CONTROL.ST:
	 * 清零停止 TX DMA 取描述符。当前裸机版本不等待 TX FIFO drain，
	 * 因为 near-end loopback 测试会在每包发送完成后才 stop。
	 */
	v = reg_rd(hal, DMA_CHAN_TX_CONTROL);
	v &= ~DMA_CONTROL_ST;
	reg_wr(hal, DMA_CHAN_TX_CONTROL, v);

	/*
	 * DMA_CHAN_RX_CONTROL.SR:
	 * 清零停止 RX DMA 写描述符/缓冲区。
	 */
	v = reg_rd(hal, DMA_CHAN_RX_CONTROL);
	v &= ~DMA_CONTROL_SR;
	reg_wr(hal, DMA_CHAN_RX_CONTROL, v);

	/*
	 * GMAC_CONFIG.TE/RE:
	 * 关闭 MAC 发送/接收状态机，防止 stop 后继续产生新的 DMA 请求。
	 */
	v = reg_rd(hal, GMAC_CONFIG);
	v &= ~(GMAC_CONFIG_TE | GMAC_CONFIG_RE);
	reg_wr(hal, GMAC_CONFIG, v);

	return 0;
}

void gmac_mac_del(void)
{
	if (!s_hal) {
		GMAC_PRINTF("EQoS HAL: gmac_mac_del — 无活动上下文，跳过\n");
		return;
	}
	GMAC_PRINTF("EQoS HAL: gmac_mac_del — 停 MAC 并清除 LM\n");
	gmac_mac_stop(s_hal);
	{
		uint32_t v = reg_rd(s_hal, GMAC_CONFIG);

		/*
		 * GMAC_CONFIG.LM:
		 * 清除 MAC 内部 loopback，使后续真实链路测试不会误用 near-end loopback 状态。
		 */
		v &= ~GMAC_CONFIG_LM;
		reg_wr(s_hal, GMAC_CONFIG, v);
	}
	s_hal = NULL;
}

int gmac_mac_transmit(gmac_hal_context_t *hal, const void *buf, uint32_t length)
{
	unsigned tries = 1000000U;
	unsigned slot = s_dma ? s_dma->tx_desc_num : EQOS_TX_RING;

	if (!hal || !s_dma || length > s_dma->buf_size || length < 16U) {
		GMAC_PRINTF("EQoS HAL: gmac_mac_transmit 参数非法 — hal=%p len=%u（需 16~%u）\n",
			    (void *)hal, (unsigned)length,
			    s_dma ? s_dma->buf_size : EQOS_BUF_SZ);
		return -1;
	}

	for (unsigned i = 0; i < s_dma->tx_desc_num; i++) {
		uint32_t d3;

		DMA_CACHE_INVALIDATE(&s_dma->tx_desc[i], EQOS_DMA_DESC_SZ);
		EQOS_MEM_BARRIER();
		d3 = s_dma->tx_desc[i].des3;
		if (!(d3 & TDES3_OWN)) {
			slot = i;
			break;
		}
	}
	if (slot >= s_dma->tx_desc_num) {
		GMAC_PRINTF("EQoS HAL: gmac_mac_transmit — TX 描述符环忙（%u 项均 OWN），丢弃\n",
			    s_dma->tx_desc_num);
		return -1;
	}

	/*
	 * 对齐 P5 gmac_esp_dma->tx_buf[slot]：
	 * 这里 s_dma->tx_buf[slot] 就是第 slot 个 TX frame buffer。
	 */
	memcpy(s_dma->tx_buf[slot], buf, length);
	DMA_CACHE_WB(s_dma->tx_buf[slot], s_dma->buf_size);
	tx_desc_prepare(&s_dma->tx_desc[slot], EQOS_VIRT_TO_PHYS(s_dma->tx_buf[slot]),
			length, length);
	EQOS_MEM_BARRIER();
	dma_prog_tx_tail(hal, slot);

	while (tries--) {
		uint32_t d3;

		DMA_CACHE_INVALIDATE(&s_dma->tx_desc[slot], EQOS_DMA_DESC_SZ);
		EQOS_MEM_BARRIER();
		d3 = s_dma->tx_desc[slot].des3;
		if (!(d3 & TDES3_OWN)) {
			if (d3 & TDES3_ERROR_SUMMARY) {
				GMAC_PRINTF("EQoS HAL: gmac_mac_transmit — TDES3 ERROR_SUMMARY，des3=0x%08x\n",
					    (unsigned)d3);
				return -1;
			}
			return 0;
		}
	}
	GMAC_PRINTF("EQoS HAL: gmac_mac_transmit — 等待 OWN 清除超时（DMA 未完成或 hang）\n");
	return -1;
}

bool gmac_get_receive_finish_int_flag(gmac_hal_context_t *hal)
{
	uint32_t st = reg_rd(hal, DMA_CHAN_STATUS);
	uint32_t en = reg_rd(hal, DMA_CHAN_INTR_ENA);

	if (st & DMA_CHAN_STATUS_RI) {
		/*
		 * DMA_CHAN_STATUS:
		 * DWMAC4 DMA channel status 位多数为 W1C。这里只回写“已置位且已使能”的位，
		 * 清除 RI/RX complete 等本轮中断状态，避免下一次轮询重复命中旧中断。
		 */
		reg_wr(hal, DMA_CHAN_STATUS, st & en);
		return true;
	}
	return false;
}

bool gmac_receive_frame(gmac_hal_context_t *hal, uint16_t *expected_frame,
			bool check_content_flag, bool return_on_every_good_pkt,
			bool timestamp_print)
{
	uint8_t tmp[EQOS_BUF_SZ];
	(void)timestamp_print;

	if (!s_dma)
		return false;

	for (unsigned k = 0; k < s_dma->rx_desc_num; k++) {
		unsigned i = (s_rx_scan + k) % s_dma->rx_desc_num;
		struct eqos_dma_desc *d = &s_dma->rx_desc[i];
		uint32_t d3;

		DMA_CACHE_INVALIDATE(d, EQOS_DMA_DESC_SZ);
		EQOS_MEM_BARRIER();
		d3 = d->des3;
		if (d3 & RDES3_OWN)
			continue;
		if (d3 & RDES3_CONTEXT_DESCRIPTOR)
			continue;
		if (!(d3 & RDES3_LAST_DESCRIPTOR))
			continue;
		if (d3 & RDES3_ERROR_SUMMARY) {
			GMAC_PRINTF("EQoS HAL: gmac_receive_frame — RDES3 ERROR_SUMMARY idx=%u des3=0x%08x\n",
				    i, (unsigned)d3);
			goto bad;
		}

		{
			unsigned flen = d3 & RDES3_PACKET_SIZE_MASK;

			if (flen < GMAC_CRC_LENGTH)
				goto bad;
			flen -= GMAC_CRC_LENGTH;
			if (flen > s_dma->buf_size)
				flen = s_dma->buf_size;
			DMA_CACHE_INVALIDATE(s_dma->rx_buf[i], s_dma->buf_size);
			memcpy(tmp, s_dma->rx_buf[i], flen);

			rx_desc_reset_owned(d, EQOS_VIRT_TO_PHYS(s_dma->rx_buf[i]));
			dma_prog_rx_tail_last(hal, i);
			s_rx_scan = (i + 1U) % s_dma->rx_desc_num;

			{
				gmac_frame_t *pkt = (gmac_frame_t *)tmp;

				if (pkt->proto != GMAC_NTOHS(GMAC_MY_NORMAL_PKT_TYPE))
					return false;
				if (expected_frame)
					(*expected_frame)++;
				if (check_content_flag) {
					if (flen < ETH_HEADER_LEN)
						return false;
					for (unsigned j = 0; j < flen - ETH_HEADER_LEN;
					     j++) {
						if (pkt->data[j] != (uint8_t)(j & 0xff)) {
							GMAC_PRINTF("payload mismatch @%u\n",
								    j);
							return false;
						}
					}
				}
				if (return_on_every_good_pkt)
					return true;
				return false;
			}
		}
bad:
		rx_desc_reset_owned(d, EQOS_VIRT_TO_PHYS(s_dma->rx_buf[i]));
		dma_prog_rx_tail_last(hal, i);
		s_rx_scan = (i + 1U) % s_dma->rx_desc_num;
	}
	return false;
}

void gmac_eqos_test_bind(uintptr_t csr_base, unsigned csr_clock_hz)
{
	s_test_csr = csr_base;
	s_test_csr_hz = csr_clock_hz;
	GMAC_PRINTF("EQoS test: bind csr_base=%p csr_clock_hz=%u（用于 1us 计数器等）\n",
		    (void *)csr_base, csr_clock_hz);
}

void test_mac_near_end_loopback_force_link(gmac_speed_t force_link_speed)
{
	static gmac_hal_context_t ctx;
	const uint16_t test_len_max = 1514U;
	static const uint16_t test_len[] = {64, 128, 256, 512, 1024, 1514};
	uint8_t *pkt;

	GMAC_PRINTF("\n======== EQoS test_mac_near_end_loopback_force_link 开始 ========\n");
	GMAC_PRINTF("EQoS test: 目标 MAC 速率=%s（与 PHY force 对齐）\n",
		    eqos_speed_str(force_link_speed));

	ctx.csr_base = s_test_csr;
	if (ctx.csr_base == 0) {
		GMAC_PRINTF("EQoS test: 错误 — 请先调用 gmac_eqos_test_bind(csr_base, csr_hz)\n");
		return;
	}
	gmac_eqos_set_csr_clock_hz(s_test_csr_hz);
	GMAC_PRINTF("EQoS test: CSR 时钟已登记 %u Hz\n", s_test_csr_hz);

	/*
	 * 先创建/绑定 DMA handle，后面用 s_dma->tx_buf[0] 准备测试帧。
	 * 这对应 P5 里 gmac_esp_new_dma() 后访问 gmac_esp_dma->tx_buf[] 的习惯。
	 */
	s_dma = gmac_eqos_new_dma();
	pkt = (uint8_t *)s_dma->tx_buf[0];

	memset(pkt, 0, test_len_max);
	{
		gmac_frame_t *f = (gmac_frame_t *)pkt;

		f->proto = GMAC_HTONS(GMAC_MY_NORMAL_PKT_TYPE);
		{
			const uint8_t src[] = { MAC_TEST_SRC_ADDR };

			memcpy(f->src, src, 6);
		}
		memset(f->dest, 0xff, 6);
		for (int i = 0; i < (int)(test_len_max - ETH_HEADER_LEN); i++)
			f->data[i] = (uint8_t)(i & 0xff);
	}

	{
		phy_extra_config_t phy_extra = {
			.force_link_speed = force_link_speed,
			.force_duplex = GMAC_DUPLEX_FULL,
			.phy_loopback_en = false,
			.is_phy_near_end_loopback = false,
		};

		GMAC_PRINTF("EQoS test: 阶段 1 — 板级/MDIO/PHY（gmac_glb_cfg_init + vcs8541_init）\n");
		gmac_glb_cfg_init(&ctx);
		gmac_phy_vcs8541_init(&g_eqos_phy_config, &ctx, &phy_extra);
	}
	GMAC_PRINTF("EQoS test: 阶段 2 — MAC/DMA 初始化 + 近端环回 + 速率/双工 + 启动\n");
	gmac_mac_init(&ctx);
	gmac_mac_near_loopback_prepare(&ctx);
	gmac_mac_set_speed(&ctx, force_link_speed);
	gmac_mac_set_duplex(&ctx, GMAC_DUPLEX_FULL);
	gmac_mac_start(&ctx);

	GMAC_PRINTF("EQoS test: 阶段 3 — 多包长环回发送/校验（EtherType 0x%04x）\n",
		    (unsigned)GMAC_MY_NORMAL_PKT_TYPE);
	for (size_t ti = 0; ti < sizeof(test_len) / sizeof(test_len[0]); ti++) {
		for (unsigned rep = 0; rep < 2U; rep++) {
			int timeout = 200;

			GMAC_PRINTF("loopback len=%u rep=%u\n", test_len[ti],
				    rep + 1U);
			if (gmac_mac_transmit(&ctx, pkt, test_len[ti]) != 0) {
				GMAC_PRINTF("TX fail\n");
				continue;
			}
			while (timeout--) {
				if (gmac_get_receive_finish_int_flag(&ctx))
					(void)0;
				if (gmac_receive_frame(&ctx, NULL, true, true,
						       false)) {
					GMAC_PRINTF("len %u PASS\n", test_len[ti]);
					break;
				}
			}
			if (timeout <= 0)
				GMAC_PRINTF("len %u timeout\n", test_len[ti]);
		}
	}

	GMAC_PRINTF("EQoS test: 阶段 4 — 停机与 PHY deinit\n");
	gmac_mac_stop(&ctx);
	gmac_phy_802_3_basic_phy_deinit(&g_eqos_phy_config, &ctx);
	gmac_mac_del();
	GMAC_PRINTF("======== EQoS test_mac_near_end_loopback_force_link 结束 ========\n\n");
}
