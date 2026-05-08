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

static const char *eqos_link_str(int link)
{
	return link == GMAC_LINK_UP ? "UP" : "DOWN";
}

/*
 * 对齐 P5:
 *   ut_isr_alloc(source, priority, handler, param)
 *   ut_isr_free(source)
 *
 * 真实板级请在 BSP 提供这些符号的强实现；当前工程先在 board_stub 提供 weak 空实现，
 * 以便测试代码在无 IRQ 环境下仍可编译和跑轮询降级路径。
 */
extern void intr_handler_set(int source, void (*isr_handle)(void *), void *param);
extern void esprv_intc_int_set_priority(int source, int priority);
extern void esprv_intc_int_enable(int source);
extern void esprv_intc_int_disable(int source);

#ifndef ETS_GMAC_SBD_INTR_INUM
#define ETS_GMAC_SBD_INTR_INUM 0
#endif

static volatile bool g_gmac_packet_received;

void ut_isr_alloc(int source, int priority, void (*isr_handle)(void *), void *param)
{
	intr_handler_set(source, isr_handle, param);
	esprv_intc_int_set_priority(source, priority);
	esprv_intc_int_enable(source);
}

void ut_isr_free(int source)
{
	esprv_intc_int_disable(source);
	intr_handler_set(source, NULL, NULL);
}

void gmac_isr_default_handler(void *args)
{
	gmac_hal_context_t *hal = (gmac_hal_context_t *)args;
	uint32_t intr_stat;
	uint32_t en;

	/*
	 * DWMAC4/EQoS 这里用 DMA_CHAN_STATUS + DMA_CHAN_INTR_ENA 获取并清中断。
	 * 与 P5 emac_hal_get_intr_status + clear_corresponding_intr 的语义一致。
	 */
	intr_stat = gmac_io_read32(hal->csr_base + (uintptr_t)DMA_CHAN_STATUS);
	en = gmac_io_read32(hal->csr_base + (uintptr_t)DMA_CHAN_INTR_ENA);
	intr_stat &= en;
	if (intr_stat)
		gmac_io_write32(hal->csr_base + (uintptr_t)DMA_CHAN_STATUS, intr_stat); /* W1C */

	if (intr_stat & DMA_CHAN_STATUS_TI)
		GMAC_PRINTF("EQoS ISR: Tx int complete\n");
	if (intr_stat & DMA_CHAN_STATUS_RI)
		g_gmac_packet_received = true;
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

unsigned gmac_eqos_get_csr_clock_hz(void)
{
	return s_csr_hz;
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
		 * GMAC4_MAC_ONEUS_TIC_COUNTER: 告诉mac多少个 CSR clock cycle 等于 1 微秒
		 * 写入 “CSR 时钟周期数 - 1”，让 MAC 内部 1us tick 与实际 CSR clock 对齐。
		 * PTP/部分超时计数会依赖这个值；不知道 CSR 时钟时跳过（e.g.:可以设置为 0），避免写错。
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
	gmac_eqos_set_csr_clock_hz(csr_clock_hz);
	GMAC_PRINTF("EQoS test: bind csr_base=%p csr_clock_hz=%u（用于 1us 计数器等）\n",
		    (void *)csr_base, csr_clock_hz);
}

void test_gmac_env_basic(uintptr_t csr_base, unsigned csr_clock_hz)
{
	static gmac_hal_context_t ctx;
	uint32_t version;
	uint32_t hw_feature0;
	uint32_t hw_feature1;
	uint32_t hw_feature2;
	uint32_t hw_feature3;
	uint32_t phyif;
	uint32_t vlan_old;
	uint32_t vlan_test = 0x55aaU;
	uint32_t vlan_readback;
	uint8_t snpsver;
	uint8_t userver;

	GMAC_PRINTF("\n======== GMAC env basic test (EQoS baremetal) 开始 ========\n");

	/*
	 * 让 env basic test 自己完成测试环境绑定，调用侧只需要传入 CSR 基址和 CSR 时钟。
	 * 等价于用户手动先调用：
	 *   gmac_eqos_test_bind(csr_base, csr_clock_hz);
	 * 再进入环境检查。
	 */
	gmac_eqos_test_bind(csr_base, csr_clock_hz);

	ctx.csr_base = s_test_csr;
	if (ctx.csr_base == 0) {
		GMAC_PRINTF("EQoS env: 错误 — csr_base 为 0，请传入有效 EQoS CSR 基址\n");
		return;
	}

	/*
	 * 对齐 P5 TEST_CASE(\"GMAC env basic test\", \"[P5 FPGA]\") 的第一步：
	 * 确认 GMAC CSR 窗口可访问，读取 version / feature 等只读关键信息。
	 */
	GMAC_PRINTF("EQoS env: csr_base=%p csr_clock_hz=%u\n",
		    (void *)ctx.csr_base, s_test_csr_hz);
	GMAC_PRINTF("EQoS env: GMAC4_VERSION addr=%p, GMAC_HW_FEATURE0 addr=%p\n",
		    (void *)(ctx.csr_base + GMAC4_VERSION),
		    (void *)(ctx.csr_base + GMAC_HW_FEATURE0));

	version = reg_rd(&ctx, GMAC4_VERSION);
	snpsver = (uint8_t)(version & 0xffU);
	userver = (uint8_t)((version >> 8) & 0xffU);
	GMAC_PRINTF("EQoS env: GMAC4_VERSION=0x%08lx, SNPSVER=0x%02x, USERVER=0x%02x\n",
		    (unsigned long)version, snpsver, userver);
	if (version == 0 || version == 0xffffffffU) {
		GMAC_PRINTF("EQoS env: FAIL — version 读值异常，检查 csr_base/时钟/复位/MMIO\n");
		return;
	}
	if (snpsver != 0x54U) {
		GMAC_PRINTF("EQoS env: WARN — 期望 EQoS/DWMAC 5.40a SNPSVER≈0x54，当前=0x%02x\n",
			    snpsver);
	} else {
		GMAC_PRINTF("EQoS env: PASS — SNPSVER 匹配 5.40a\n");
	}

	/*
	 * Hardware Feature 寄存器是只读能力描述：
	 * - FEATURE0 常见包含 GMII/RGMII、checksum、multi-MAC、TSO/VLAN 等能力。
	 * - FEATURE1/2/3 常见包含 FIFO/queue/channel/PTP/PPS/5.x 扩展能力。
	 * 这里只打印原始值，具体 bit 请对照 Linux dwmac4_dma.c / dwmac4.h 或 databook。
	 */
	hw_feature0 = reg_rd(&ctx, GMAC_HW_FEATURE0);
	hw_feature1 = reg_rd(&ctx, GMAC_HW_FEATURE1);
	hw_feature2 = reg_rd(&ctx, GMAC_HW_FEATURE2);
	hw_feature3 = reg_rd(&ctx, GMAC_HW_FEATURE3);
	GMAC_PRINTF("EQoS env: HW_FEATURE0=0x%08lx\n", (unsigned long)hw_feature0);
	GMAC_PRINTF("EQoS env: HW_FEATURE1=0x%08lx\n", (unsigned long)hw_feature1);
	GMAC_PRINTF("EQoS env: HW_FEATURE2=0x%08lx\n", (unsigned long)hw_feature2);
	GMAC_PRINTF("EQoS env: HW_FEATURE3=0x%08lx\n", (unsigned long)hw_feature3);

	phyif = reg_rd(&ctx, GMAC_PHYIF_CONTROL_STATUS);
	GMAC_PRINTF("EQoS env: GMAC_PHYIF_CONTROL_STATUS=0x%08lx（RGMII link/update 等状态）\n",
		    (unsigned long)phyif);

	/*
	 * 可恢复寄存器读写测试：
	 * P5 用 vlan_tag_reg 写 0x55aa 后读回再恢复。DWMAC4/EQoS 中 VLAN_TAG offset
	 * 为 0x50（Linux stmmac_vlan.h: VLAN_TAG），这里采用同样思路验证 MAC CSR 写通路。
	 *
	 * 注意：该测试会短暂改写 VLAN_TAG，因此建议在 MAC 未启动收发时调用；
	 * 函数最后会把旧值写回。
	 */
	vlan_old = reg_rd(&ctx, GMAC_VLAN_TAG);
	GMAC_PRINTF("EQoS env: VLAN_TAG old=0x%08lx，写入测试值 0x%08lx\n",
		    (unsigned long)vlan_old, (unsigned long)vlan_test);
	reg_wr(&ctx, GMAC_VLAN_TAG, vlan_test);
	vlan_readback = reg_rd(&ctx, GMAC_VLAN_TAG);
	if (vlan_readback == vlan_test) {
		GMAC_PRINTF("EQoS env: PASS — VLAN_TAG write/readback OK (0x%08lx)\n",
			    (unsigned long)vlan_readback);
	} else {
		GMAC_PRINTF("EQoS env: FAIL — VLAN_TAG readback=0x%08lx, expected=0x%08lx\n",
			    (unsigned long)vlan_readback, (unsigned long)vlan_test);
	}
	reg_wr(&ctx, GMAC_VLAN_TAG, vlan_old);
	GMAC_PRINTF("EQoS env: VLAN_TAG restored=0x%08lx\n",
		    (unsigned long)reg_rd(&ctx, GMAC_VLAN_TAG));

	GMAC_PRINTF("======== GMAC env basic test (EQoS baremetal) 结束 ========\n\n");
}

void test_get_phy_addr(uintptr_t csr_base, unsigned csr_clock_hz)
{
	static gmac_hal_context_t ctx;

	GMAC_PRINTF("\n======== GET_PHY_ADDR test (EQoS baremetal) 开始 ========\n");
	gmac_eqos_test_bind(csr_base, csr_clock_hz);
	ctx.csr_base = s_test_csr;
	if (ctx.csr_base == 0) {
		GMAC_PRINTF("EQoS GET_PHY_ADDR: 错误 — csr_base 为 0\n");
		return;
	}

	/*
	 * 对齐 P5 GET_PHY_ADDR test：
	 * 先做全局/板级/MDIO 初始化，再通过 Clause22 IDR1/IDR2 扫描 PHY 地址。
	 */
	gmac_glb_cfg_init(&ctx);
	if (g_eqos_phy_config.addr == GMAC_PHY_ADDR_AUTO) {
		if (gmac_phy_detect_phy_addr(&ctx, &g_eqos_phy_config.addr)) {
			GMAC_PRINTF("EQoS GET_PHY_ADDR: FAIL — 未找到 PHY\n");
			GMAC_PRINTF("======== GET_PHY_ADDR test (EQoS baremetal) 结束 ========\n\n");
			return;
		}
	}
	if (g_eqos_phy_config.addr != GMAC_PHY_ADDR_AUTO) {
		GMAC_PRINTF("EQoS GET_PHY_ADDR: Updated PHY address = %d\n",
			    g_eqos_phy_config.addr);
	}
	GMAC_PRINTF("======== GET_PHY_ADDR test (EQoS baremetal) 结束 ========\n\n");
}

void test_smi_phy_reg_read_write(uintptr_t csr_base, unsigned csr_clock_hz)
{
	static gmac_hal_context_t ctx;

	GMAC_PRINTF("\n======== SMI_INTF:PHY_REG_READ_WRITE test (EQoS baremetal) 开始 ========\n");
	gmac_eqos_test_bind(csr_base, csr_clock_hz);
	ctx.csr_base = s_test_csr;
	if (ctx.csr_base == 0) {
		GMAC_PRINTF("EQoS SMI: 错误 — csr_base 为 0\n");
		return;
	}

	gmac_glb_cfg_init(&ctx);
	if (gmac_phy_vcs8541_reg_read_write(&g_eqos_phy_config, &ctx))
		GMAC_PRINTF("EQoS SMI: FAIL — gmac_phy_vcs8541_reg_read_write failed\n");

	GMAC_PRINTF("======== SMI_INTF:PHY_REG_READ_WRITE test (EQoS baremetal) 结束 ========\n\n");
}

void test_phy_auto_negotiation_link_partner(uintptr_t csr_base, unsigned csr_clock_hz)
{
	static gmac_hal_context_t ctx;
	gmac_port_link_t link_status = { GMAC_LINK_DOWN, GMAC_SPEED_10M, GMAC_DUPLEX_HALF };
	int link_status_old = GMAC_LINK_DOWN;
	unsigned time_cnt = 0;

	GMAC_PRINTF("\n======== PHY auto_negotiation test(link_partner) (EQoS baremetal) 开始 ========\n");
	GMAC_PRINTF("EQoS AN: 请连接真实 link partner（交换机/PC/对端 PHY），本测试最多观察约 10 秒 link-up 状态\n");
	gmac_eqos_test_bind(csr_base, csr_clock_hz);
	ctx.csr_base = s_test_csr;
	if (ctx.csr_base == 0) {
		GMAC_PRINTF("EQoS AN: 错误 — csr_base 为 0\n");
		return;
	}

	gmac_glb_cfg_init(&ctx);
	gmac_phy_vcs8541_init(&g_eqos_phy_config, &ctx, NULL);
	if (g_eqos_phy_config.addr == GMAC_PHY_ADDR_AUTO) {
		GMAC_PRINTF("EQoS AN: FAIL — PHY init 后仍未获得 PHY 地址\n");
		return;
	}

	if (gmac_phy_vcs8541_auto_nego_restart(&g_eqos_phy_config, &ctx))
		GMAC_PRINTF("EQoS AN: WARN — auto-negotiation 未在超时时间内完成，继续观察 link 状态\n");

	GMAC_PRINTF("----------- GMAC PHY wait for connection -------\n");
	while (time_cnt < 15U) {
		if (gmac_phy_vcs8541_get_link_info(&g_eqos_phy_config, &ctx, &link_status)) {
			GMAC_PRINTF("EQoS AN: WARN — read link info failed\n");
		} else if (link_status_old != link_status.link_st) {
			link_status_old = link_status.link_st;
			GMAC_PRINTF("EQoS AN: link %s, speed=%s, duplex=%s\n",
				    eqos_link_str(link_status.link_st),
				    eqos_speed_str(link_status.speed),
				    link_status.duplex == GMAC_DUPLEX_FULL ? "full" : "half");
		} else if (link_status.link_st == GMAC_LINK_UP) {
			GMAC_PRINTF("EQoS AN: link still UP (%u/10 sec), speed=%s, duplex=%s\n",
				    time_cnt, eqos_speed_str(link_status.speed),
				    link_status.duplex == GMAC_DUPLEX_FULL ? "full" : "half");
			if (time_cnt >= 10U)
				break;
		}

		gmac_board_delay_us(1000000U);
		time_cnt++;
	}

	GMAC_PRINTF("======== PHY auto_negotiation test(link_partner) (EQoS baremetal) 结束 ========\n\n");
}

void test_mac_near_end_loopback_force_link(uintptr_t csr_base, unsigned csr_clock_hz,
					   gmac_speed_t force_link_speed)
{
	static gmac_hal_context_t ctx;
	const uint16_t test_len_max = 1514U;
	static const uint16_t test_len[] = {64, 128, 256, 512, 1024, 1514};
	uint8_t *pkt;

	GMAC_PRINTF("\n======== EQoS test_mac_near_end_loopback_force_link 开始 ========\n");
	GMAC_PRINTF("EQoS test: 目标 MAC 速率=%s（与 PHY force 对齐）\n",
		    eqos_speed_str(force_link_speed));

	/*
	 * 让 loopback test 自己完成测试环境绑定，调用侧只需要传入 CSR 基址、
	 * CSR 时钟和目标速率，不再需要提前单独调用 gmac_eqos_test_bind()。
	 */
	gmac_eqos_test_bind(csr_base, csr_clock_hz);

	ctx.csr_base = s_test_csr;
	if (ctx.csr_base == 0) {
		GMAC_PRINTF("EQoS test: 错误 — csr_base 为 0，请传入有效 EQoS CSR 基址\n");
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

/*
 * 对齐 P5 test_phy_near_end_loopback_force_link(force_link_speed)：
 * emac tx -> PHY RX -> PHY 近端环回 -> PHY TX -> emac rx。
 * 裸机入口额外带 csr_base/csr_clock_hz，并在函数内部完成测试环境绑定。
 */
void test_phy_near_end_loopback_force_link(uintptr_t csr_base, unsigned csr_clock_hz,
					   gmac_speed_t force_link_speed)
{
	static gmac_hal_context_t ctx;
	const uint16_t test_len_max = 1514U;
	static const uint16_t test_len[] = {64, 128, 256, 512, 768, 1024, 1280, 1466, 1514};
	uint8_t *pkt;
	phy_extra_config_t phy_extra = {
		.force_link_speed = force_link_speed,
		.force_duplex = GMAC_DUPLEX_FULL,
		.phy_loopback_en = true,
		.is_phy_near_end_loopback = true,
	};

	GMAC_PRINTF("\n======== EQoS test_phy_near_end_loopback_force_link 开始 ========\n");
	GMAC_PRINTF("EQoS PHY-LB: force speed=%s, PHY near-end loopback enable\n",
		    eqos_speed_str(force_link_speed));

	/*
	 * 让 PHY near-end loopback test 自己完成环境绑定，调用侧只需要传入
	 * CSR 基址、CSR 时钟和目标速率。
	 */
	gmac_eqos_test_bind(csr_base, csr_clock_hz);

	ctx.csr_base = s_test_csr;
	if (ctx.csr_base == 0) {
		GMAC_PRINTF("EQoS PHY-LB: 错误 — csr_base 为 0，请传入有效 EQoS CSR 基址\n");
		return;
	}
	gmac_eqos_set_csr_clock_hz(s_test_csr_hz);
	GMAC_PRINTF("EQoS PHY-LB: 使用已绑定 CSR base=%p, csr_clock_hz=%u\n",
		    (void *)ctx.csr_base, s_test_csr_hz);

	s_dma = gmac_eqos_new_dma();
	pkt = (uint8_t *)s_dma->tx_buf[0];

	memset(pkt, 0, test_len_max);
	{
		gmac_frame_t *f = (gmac_frame_t *)pkt;
		const uint8_t src[] = { MAC_TEST_SRC_ADDR };

		f->proto = GMAC_HTONS(GMAC_MY_NORMAL_PKT_TYPE);
		memcpy(f->src, src, sizeof(f->src));
		memset(f->dest, 0xff, sizeof(f->dest));
		for (int i = 0; i < (int)(test_len_max - ETH_HEADER_LEN); ++i)
			f->data[i] = (uint8_t)(i & 0xff);
	}
	GMAC_PRINTF("EQoS PHY-LB: transmit content ready\n");

	GMAC_PRINTF("EQoS PHY-LB: 阶段 1 — 板级/MDIO/PHY force link + PHY 近端环回\n");
	gmac_glb_cfg_init(&ctx);
	gmac_phy_vcs8541_init(&g_eqos_phy_config, &ctx, &phy_extra);

	GMAC_PRINTF("EQoS PHY-LB: 阶段 2 — MAC/DMA 初始化 + 速率/双工 + 启动\n");
	gmac_mac_init(&ctx);
	gmac_mac_set_speed(&ctx, phy_extra.force_link_speed);
	gmac_mac_set_duplex(&ctx, phy_extra.force_duplex);
	gmac_mac_start(&ctx);

	/* 对齐 P5：等待 PHY 近端环回与 MAC 数据路径稳定，避免起始几个包丢失。 */
	gmac_board_delay_us(100000U);

	GMAC_PRINTF("EQoS PHY-LB: 阶段 3 — 多包长 PHY 近端环回发送/校验\n");
	for (size_t i = 0; i < sizeof(test_len) / sizeof(test_len[0]); i++) {
		for (unsigned j = 0; j < 2U; j++) {
			int check_recv_timeout = 100;

			GMAC_PRINTF("len:%u _broadcast send %u times!\n",
				    test_len[i], j + 1U);
			if (gmac_mac_transmit(&ctx, pkt, test_len[i]) != 0) {
				GMAC_PRINTF("EQoS PHY-LB: TX fail len=%u\n", test_len[i]);
				continue;
			}
			gmac_board_delay_us(20000U);

			while (check_recv_timeout--) {
				if (gmac_get_receive_finish_int_flag(&ctx) &&
				    gmac_receive_frame(&ctx, NULL, true, true, false)) {
					GMAC_PRINTF("len:%u phy-nearend-loopback pass !!!\n",
						    test_len[i]);
					break;
				}
				gmac_board_delay_us(20000U);
			}
			if (check_recv_timeout <= 0) {
				GMAC_PRINTF("Fail: phy-near end loopback received timeout, "
					    "check_recv_timeout=%d !!!\n",
					    check_recv_timeout);
			}
		}
	}

	GMAC_PRINTF("EQoS PHY-LB: 阶段 4 — 停机与 PHY deinit\n");
	gmac_mac_stop(&ctx);
	gmac_phy_802_3_basic_phy_deinit(&g_eqos_phy_config, &ctx);
	gmac_mac_del();
	GMAC_PRINTF("======== EQoS test_phy_near_end_loopback_force_link 结束 ========\n\n");
}

/*
 * 对齐 P5:
 * TEST_CASE("GMAC Interrupt test(receive pkt,force_link_mac)", "[P5 FPGA]")
 *
 * 当前 EQoS 版保留同样的核心验证点：
 * - ut_isr_alloc 注册默认 ISR
 * - 发包后等待 ISR 设置 g_gmac_packet_received
 * - 再进入 RX 描述符解析校验
 */
void test_gmac_interrupt_receive_pkt_force_link_mac(uintptr_t csr_base, unsigned csr_clock_hz)
{
	static gmac_hal_context_t ctx;
	const uint16_t test_len_max = 1514U;
	static const uint16_t test_len[] = {64, 128, 256, 512, 1024, 1514};
	const unsigned send_times_every_len = 2U;
	int recv_isr_wait_us;
	gmac_speed_t force_link_speed;
	uint8_t *pkt;
	phy_extra_config_t phy_extra;

#if (GMAC_PHY_INTF == EQOS_PHY_INTF_RGMII)
	force_link_speed = GMAC_SPEED_1000M;
#else
	force_link_speed = GMAC_SPEED_100M;
#endif
	recv_isr_wait_us = (force_link_speed == GMAC_SPEED_1000M) ? 20 : 200;

	phy_extra.force_link_speed = force_link_speed;
	phy_extra.force_duplex = GMAC_DUPLEX_FULL;
	phy_extra.phy_loopback_en = false;
	phy_extra.is_phy_near_end_loopback = false;

	GMAC_PRINTF("\n======== GMAC Interrupt test(receive pkt,force_link_mac) 开始 ========\n");
	GMAC_PRINTF("EQoS IRQ test: force speed=%s, recv_isr_wait_us=%d\n",
		    eqos_speed_str(force_link_speed), recv_isr_wait_us);

	gmac_eqos_test_bind(csr_base, csr_clock_hz);
	ctx.csr_base = s_test_csr;
	if (ctx.csr_base == 0) {
		GMAC_PRINTF("EQoS IRQ test: 错误 — csr_base 为 0，请传入有效 EQoS CSR 基址\n");
		return;
	}
	gmac_eqos_set_csr_clock_hz(s_test_csr_hz);

	s_dma = gmac_eqos_new_dma();
	pkt = (uint8_t *)s_dma->tx_buf[0];
	memset(pkt, 0, test_len_max);
	{
		gmac_frame_t *f = (gmac_frame_t *)pkt;
		const uint8_t src[] = { MAC_TEST_SRC_ADDR };

		f->proto = GMAC_HTONS(GMAC_MY_NORMAL_PKT_TYPE);
		memcpy(f->src, src, sizeof(f->src));
		memset(f->dest, 0xff, sizeof(f->dest));
		for (int i = 0; i < (int)(test_len_max - ETH_HEADER_LEN); ++i)
			f->data[i] = (uint8_t)(i & 0xff);
	}
	GMAC_PRINTF("EQoS IRQ test: transmit content ready\n");

	ut_isr_alloc(ETS_GMAC_SBD_INTR_INUM, 1, gmac_isr_default_handler, (void *)(&ctx));

	gmac_glb_cfg_init(&ctx);
	gmac_phy_vcs8541_init(&g_eqos_phy_config, &ctx, &phy_extra);
	gmac_mac_init(&ctx);
	gmac_mac_near_loopback_prepare(&ctx);
	gmac_mac_set_speed(&ctx, force_link_speed);
	gmac_mac_set_duplex(&ctx, GMAC_DUPLEX_FULL);
	gmac_mac_start(&ctx);

	gmac_board_delay_us(100000U);

	for (size_t i = 0; i < sizeof(test_len) / sizeof(test_len[0]); i++) {
		for (unsigned j = 0; j < send_times_every_len; j++) {
			int wait_isr = recv_isr_wait_us;
			int check_recv_timeout = 100;

			GMAC_PRINTF("len:%u _broadcast send %u/%u times!\n",
				    test_len[i], j + 1U, send_times_every_len);
			g_gmac_packet_received = false;
			(void)gmac_mac_transmit(&ctx, pkt, (uint32_t)test_len[i]);

			/* 等 ISR 置位；在未真正接 IRQ 的环境下退化为轮询 status。 */
			while (!g_gmac_packet_received && (wait_isr-- > 0)) {
				if (gmac_get_receive_finish_int_flag(&ctx))
					g_gmac_packet_received = true;
				else
					gmac_board_delay_us(1U);
			}
			if (!g_gmac_packet_received) {
				GMAC_PRINTF("interrupt rx timeout,len:%u GMAC Interrupt test err !!!\n",
					    test_len[i]);
				continue;
			}

			while (check_recv_timeout--) {
				if (gmac_receive_frame(&ctx, NULL, true, true, false)) {
					GMAC_PRINTF("len:%u interrupt receive pass !!!\n", test_len[i]);
					break;
				}
				gmac_board_delay_us(20000U);
			}
			if (check_recv_timeout <= 0) {
				GMAC_PRINTF("Fail: interrupt receive frame timeout len=%u !!!\n",
					    test_len[i]);
			}
		}
	}

	ut_isr_free(ETS_GMAC_SBD_INTR_INUM);
	gmac_mac_stop(&ctx);
	gmac_phy_802_3_basic_phy_deinit(&g_eqos_phy_config, &ctx);
	gmac_mac_del();
	GMAC_PRINTF("======== GMAC Interrupt test(receive pkt,force_link_mac) 结束 ========\n\n");
}

static void eqos_prepare_broadcast_test_packet(uint8_t *pkt, uint16_t test_len_max)
{
	gmac_frame_t *f = (gmac_frame_t *)pkt;
	const uint8_t src[] = { MAC_TEST_SRC_ADDR };

	memset(pkt, 0, test_len_max);
	f->proto = GMAC_HTONS(GMAC_MY_NORMAL_PKT_TYPE);
	memcpy(f->src, src, sizeof(f->src));
	memset(f->dest, 0xff, sizeof(f->dest));
	for (int i = 0; i < (int)(test_len_max - ETH_HEADER_LEN); ++i)
		f->data[i] = (uint8_t)(i & 0xff);
}

static void eqos_stop_and_deinit(gmac_hal_context_t *ctx)
{
	gmac_mac_stop(ctx);
	gmac_phy_802_3_basic_phy_deinit(&g_eqos_phy_config, ctx);
	gmac_mac_del();
}

static int eqos_setup_link_partner(gmac_hal_context_t *ctx, uintptr_t csr_base,
				   unsigned csr_clock_hz, bool start_mac_after_link)
{
	gmac_port_link_t link_status = { GMAC_LINK_DOWN, GMAC_SPEED_100M, GMAC_DUPLEX_FULL };
	unsigned sec = 0;

	gmac_eqos_test_bind(csr_base, csr_clock_hz);
	ctx->csr_base = s_test_csr;
	if (ctx->csr_base == 0) {
		GMAC_PRINTF("EQoS: 错误 — csr_base 为 0\n");
		return -1;
	}
	gmac_eqos_set_csr_clock_hz(s_test_csr_hz);

	s_dma = gmac_eqos_new_dma();

	gmac_glb_cfg_init(ctx);
	gmac_phy_vcs8541_init(&g_eqos_phy_config, ctx, NULL);
	gmac_mac_init(ctx);

	if (gmac_phy_vcs8541_auto_nego_restart(&g_eqos_phy_config, ctx))
		GMAC_PRINTF("EQoS: WARN — auto-negotiation 超时，继续等待 link\n");

	for (sec = 0; sec < 15U; sec++) {
		if (!gmac_phy_vcs8541_get_link_info(&g_eqos_phy_config, ctx, &link_status) &&
		    link_status.link_st == GMAC_LINK_UP)
			break;
		gmac_board_delay_us(1000000U);
	}
	if (link_status.link_st != GMAC_LINK_UP) {
		GMAC_PRINTF("EQoS: FAIL — link partner 未在超时内 link-up\n");
		return -1;
	}

	if (start_mac_after_link) {
		gmac_mac_set_speed(ctx, link_status.speed);
		gmac_mac_set_duplex(ctx, link_status.duplex);
		gmac_mac_start(ctx);
	}
	return 0;
}

static int eqos_setup_force_link_phy_near_loopback(gmac_hal_context_t *ctx, uintptr_t csr_base,
						    unsigned csr_clock_hz, gmac_speed_t speed)
{
	phy_extra_config_t phy_extra = {
		.force_link_speed = speed,
		.force_duplex = GMAC_DUPLEX_FULL,
		.phy_loopback_en = true,
		.is_phy_near_end_loopback = true,
	};

	gmac_eqos_test_bind(csr_base, csr_clock_hz);
	ctx->csr_base = s_test_csr;
	if (ctx->csr_base == 0) {
		GMAC_PRINTF("EQoS: 错误 — csr_base 为 0\n");
		return -1;
	}
	gmac_eqos_set_csr_clock_hz(s_test_csr_hz);

	s_dma = gmac_eqos_new_dma();
	gmac_glb_cfg_init(ctx);
	gmac_phy_vcs8541_init(&g_eqos_phy_config, ctx, &phy_extra);
	gmac_mac_init(ctx);
	gmac_mac_set_speed(ctx, speed);
	gmac_mac_set_duplex(ctx, GMAC_DUPLEX_FULL);
	gmac_mac_start(ctx);
	return 0;
}

/*
 * here not linkpartner, need to configure phy forced link(e.g.1000Mbps, let PHY generate RX clock)
 * emac tx -> phyrx -> phytx -> emac rx
 * debug/determine rgmii txclk_delay & rxclk_delay or tx/rx reverse parameter
 */
void test_debug_determine_rgmii_1000m_tx_rx_clk_delay_or_reverse_para_base_on_phy_near_end_loopback_force_link(
	uintptr_t csr_base, unsigned csr_clock_hz)
{
	static gmac_hal_context_t ctx;
	const uint16_t test_len_max = 1514U;
	static const uint16_t test_len[] = {64, 128, 256, 512, 768, 1024, 1280, 1466, 1514};
	uint8_t *pkt;
	const uint8_t rev_flags[2] = {0, 1};
	const uint8_t delays[8] = {0, 1, 2, 3, 4, 5, 6, 7};
	uint8_t pass_num = 0;
	uint8_t pass_combo_para[64] = {0};
	bool fail_flag = false;

	GMAC_PRINTF("\n======== debug/determine rgmii_1000m tx/rx clk_delay or reverse para base on phy near end loopback(force_link) 开始 ========\n");

	s_dma = gmac_eqos_new_dma();
	pkt = (uint8_t *)s_dma->tx_buf[0];
	eqos_prepare_broadcast_test_packet(pkt, test_len_max);

	for (unsigned tri = 0; tri < 2; tri++) {
		for (unsigned trd = 0; trd < 8; trd++) {
			for (unsigned rri = 0; rri < 2; rri++) {
				for (unsigned rrd = 0; rrd < 8; rrd++) {
					GMAC_PRINTF("EQoS RGMII scan: tx_rev=%u tx_delay=%u rx_rev=%u rx_delay=%u\n",
						    rev_flags[tri], delays[trd], rev_flags[rri], delays[rrd]);
					gmac_phy_set_rgmii_timing(delays[trd], delays[rrd],
								  rev_flags[tri] != 0U,
								  rev_flags[rri] != 0U);
					if (eqos_setup_force_link_phy_near_loopback(&ctx, csr_base,
										    csr_clock_hz,
										    GMAC_SPEED_1000M)) {
						eqos_stop_and_deinit(&ctx);
						continue;
					}
					gmac_board_delay_us(100000U);
					for (size_t i = 0; i < sizeof(test_len) / sizeof(test_len[0]); i++) {
						int check_recv_timeout = 5; /* 5 * 20ms = 100ms */

						fail_flag = false;
						(void)gmac_mac_transmit(&ctx, pkt, test_len[i]);
						gmac_board_delay_us(20000U);

						while (check_recv_timeout--) {
							if (gmac_get_receive_finish_int_flag(&ctx)) {
								bool recv_finish_flag =
									gmac_receive_frame(&ctx, NULL, true, true, false);

								if (recv_finish_flag) {
									break;
								}
								GMAC_PRINTF("phy-nearend-loopback fail at [x,y:m,n] = [%u,%u:%u,%u] @test_len=%u\n",
									    tri, rri, trd, rrd, test_len[i]);
								fail_flag = true;
								break;
							}
							gmac_board_delay_us(20000U);
						}

						if (check_recv_timeout <= 0) {
							fail_flag = true;
							GMAC_PRINTF("timeout: phy-nearend-loopback fail at [x,y:m,n] = [%u,%u:%u,%u] @test_len=%u\n",
								    tri, rri, trd, rrd, test_len[i]);
						}

						if (fail_flag)
							break;

						if (i == (sizeof(test_len) / sizeof(test_len[0]) - 1U)) {
							uint8_t combo = (uint8_t)((tri << 7) | (rri << 6) |
										  (trd << 3) | rrd);

							GMAC_PRINTF("phy-nearend-loopback pass at [x,y:m,n] = [%u,%u:%u,%u]\n",
								    tri, rri, trd, rrd);
							if (pass_num < sizeof(pass_combo_para)) {
								pass_combo_para[pass_num++] = combo;
								GMAC_PRINTF("pass_num=%u, pass_combo=%u\n",
									    pass_num, combo);
							}
						}
					}
					eqos_stop_and_deinit(&ctx);
				}
			}
		}
	}

	if (pass_num) {
		GMAC_PRINTF("pass num is %u\n", pass_num);
		for (unsigned i = 0; i < pass_num; i++) {
			int txinv = (pass_combo_para[i] >> 7) & 0x1;
			int rxinv = (pass_combo_para[i] >> 6) & 0x1;
			int txdelay = (pass_combo_para[i] >> 3) & 0x7;
			int rxdelay = pass_combo_para[i] & 0x7;

			GMAC_PRINTF(" [pass_combo:%u]: [(txinv,txdelay) (rxinv,rxdelay)]=[(%d:%d) (%d:%d)]\n",
				    pass_combo_para[i], txinv, txdelay, rxinv, rxdelay);
		}
	} else {
		GMAC_PRINTF(" --@@-@@-@@-@@-- RGMII tx/rx para fail all settings\n");
	}
	GMAC_PRINTF("======== rgmii_1000m tx/rx delay/reverse scan 结束 ========\n\n");
}

/*
 * 对接外部 link partner（非 PHY near-end loopback / 非 MAC internal loopback）；速度与双工由自协商决定。
 * TX：仅扫描 tx_clk_inv / emac_tx_clk_delay；对端用 Wireshark 看 RGMII 侧发包是否正常。
 */
void test_debug_determine_rgmii_tx_clk_delay_or_reverse_para_link_partner_check_on_wireshark(
	uintptr_t csr_base, unsigned csr_clock_hz)
{
	static gmac_hal_context_t ctx;
	const uint16_t test_len = 512U;
	uint8_t *pkt;
	const uint8_t rev_flags[2] = {0, 1};
	const uint8_t delays[8] = {0, 1, 2, 3, 4, 5, 6, 7};

	GMAC_PRINTF("\n======== debug/determine rgmii tx clk_delay or reverse para (link partner) (check on Wireshark) 开始 ========\n");

	s_dma = gmac_eqos_new_dma();
	pkt = (uint8_t *)s_dma->tx_buf[0];
	eqos_prepare_broadcast_test_packet(pkt, test_len);

	for (unsigned ri = 0; ri < 2; ri++) {
		for (unsigned dj = 0; dj < 8; dj++) {
			GMAC_PRINTF("EQoS RGMII TX scan: tx_rev=%u tx_delay=%u\n",
				    rev_flags[ri], delays[dj]);
			gmac_phy_set_rgmii_timing(delays[dj], 5U, rev_flags[ri] != 0U, false);
			if (eqos_setup_link_partner(&ctx, csr_base, csr_clock_hz, true)) {
				eqos_stop_and_deinit(&ctx);
				continue;
			}
			for (unsigned k = 0; k < 3U; k++) {
				(void)gmac_mac_transmit(&ctx, pkt, test_len);
				GMAC_PRINTF("TX scan burst=%u done, check Wireshark on link partner\n",
					    k + 1U);
				gmac_board_delay_us(100000U);
			}
			eqos_stop_and_deinit(&ctx);
		}
	}
	GMAC_PRINTF("======== rgmii tx delay/reverse link partner scan 结束 ========\n\n");
}

/*
 * 对接外部 link partner（非 near-end loopback）；RX 仅扫 rx_clk_inv / emac_rx_clk_delay。
 * 对端 link up 后由主机脚本发送 GMAC_MY_NORMAL_PKT_TYPE 帧；3s 内累计 >10 判该组 RX OK。
 */
void test_debug_determine_rgmii_rx_clk_delay_or_reverse_para_link_partner_frames_from_host_script(
	uintptr_t csr_base, unsigned csr_clock_hz)
{
	static gmac_hal_context_t ctx;
	const uint8_t rev_flags[2] = {0, 1};
	const uint8_t delays[8] = {0, 1, 2, 3, 4, 5, 6, 7};

	GMAC_PRINTF("\n======== debug/determine rgmii rx clk_delay or reverse para (link partner) (frames from host script) 开始 ========\n");
	GMAC_PRINTF("请由 host 脚本持续发 GMAC_MY_NORMAL_PKT_TYPE 帧\n");

	for (unsigned ri = 0; ri < 2; ri++) {
		for (unsigned dj = 0; dj < 8; dj++) {
			uint16_t expected_frame_cnt = 0;
			int recv_poll = 3000; /* 3000 * 1ms = 3s */
			bool rx_ok = false;

			GMAC_PRINTF("EQoS RGMII RX scan: rx_rev=%u rx_delay=%u\n",
				    rev_flags[ri], delays[dj]);
			gmac_phy_set_rgmii_timing(5U, delays[dj], false, rev_flags[ri] != 0U);
			if (eqos_setup_link_partner(&ctx, csr_base, csr_clock_hz, true)) {
				eqos_stop_and_deinit(&ctx);
				continue;
			}
			while (recv_poll-- > 0 && !rx_ok) {
				while (gmac_get_receive_finish_int_flag(&ctx)) {
					(void)gmac_receive_frame(&ctx, &expected_frame_cnt, false,
								 false, false);
					if (expected_frame_cnt > 10U) {
						rx_ok = true;
						break;
					}
				}
				gmac_board_delay_us(1000U);
			}
			GMAC_PRINTF("EQoS RX scan result: count=%u %s\n", expected_frame_cnt,
				    rx_ok ? "PASS" : "FAIL");
			eqos_stop_and_deinit(&ctx);
		}
	}
	GMAC_PRINTF("======== rgmii rx delay/reverse link partner scan 结束 ========\n\n");
}

/*
 * TEST_CASE("transmit_infinitely test(link_partner)", "[P5 FPGA]")
 * 对接 link partner 后持续发固定长度帧，便于压测链路稳定性/吞吐。
 */
void test_transmit_infinitely_link_partner(uintptr_t csr_base, unsigned csr_clock_hz)
{
	static gmac_hal_context_t ctx;
	const uint16_t test_len = 1514U;
	uint8_t *pkt;
	unsigned cnt = 0;

	GMAC_PRINTF("\n======== transmit_infinitely test(link_partner) 开始 ========\n");
	s_dma = gmac_eqos_new_dma();
	pkt = (uint8_t *)s_dma->tx_buf[0];
	eqos_prepare_broadcast_test_packet(pkt, test_len);

	if (eqos_setup_link_partner(&ctx, csr_base, csr_clock_hz, true))
		return;

	while (1) {
		(void)gmac_mac_transmit(&ctx, pkt, test_len);
		GMAC_PRINTF("TX infinite cnt=%u len=%u\n", ++cnt, test_len);
		gmac_board_delay_us(1000000U);
	}
}

/*
 * TEST_CASE("transmit_diff_len test(link_partner)", "[P5 FPGA]")
 * 对接 link partner 后按多包长发送，观察不同帧长在链路上的表现。
 */
void test_transmit_diff_len_link_partner(uintptr_t csr_base, unsigned csr_clock_hz)
{
	static gmac_hal_context_t ctx;
	const uint16_t test_len_max = 1514U;
	static const uint16_t test_len[] = {64, 128, 256, 512, 1024, 1514};
	const unsigned send_times_every_len = 10U;
	uint8_t *pkt;

	GMAC_PRINTF("\n======== transmit_diff_len test(link_partner) 开始 ========\n");
	s_dma = gmac_eqos_new_dma();
	pkt = (uint8_t *)s_dma->tx_buf[0];
	eqos_prepare_broadcast_test_packet(pkt, test_len_max);

	if (eqos_setup_link_partner(&ctx, csr_base, csr_clock_hz, true))
		return;

	for (size_t i = 0; i < sizeof(test_len) / sizeof(test_len[0]); i++) {
		for (unsigned j = 0; j < send_times_every_len; j++) {
			(void)gmac_mac_transmit(&ctx, pkt, test_len[i]);
			GMAC_PRINTF("tx diff len=%u %u/%u\n",
				    test_len[i], j + 1U, send_times_every_len);
			gmac_board_delay_us(20000U);
		}
	}

	eqos_stop_and_deinit(&ctx);
	GMAC_PRINTF("======== transmit_diff_len test(link_partner) 结束 ========\n\n");
}

/*
 * TEST_CASE("receive test(link_partner)", "[P5 FPGA]")
 * 对接 link partner 后持续接收并统计 GMAC_MY_NORMAL_PKT_TYPE 帧数量。
 */
void test_receive_link_partner(uintptr_t csr_base, unsigned csr_clock_hz)
{
	static gmac_hal_context_t ctx;
	uint16_t expected_frame_cnt = 0;

	GMAC_PRINTF("\n======== receive test(link_partner) 开始 ========\n");
	GMAC_PRINTF("请由对端持续发 GMAC_MY_NORMAL_PKT_TYPE 帧\n");
	if (eqos_setup_link_partner(&ctx, csr_base, csr_clock_hz, true))
		return;

	while (1) {
		while (gmac_get_receive_finish_int_flag(&ctx)) {
			(void)gmac_receive_frame(&ctx, &expected_frame_cnt, false, false, false);
		}
		GMAC_PRINTF("RX count=%u\n", expected_frame_cnt);
		gmac_board_delay_us(1000000U);
	}
}

/*
 * 测试长时间发送接收，基于 PHY Near-end loopback
 * here not linkpartner, need to configure phy forced link
 * emac tx -> phyrx -> phytx -> emac rx
 */
void test_transmit_and_receive_longtime_force_link_phy(uintptr_t csr_base, unsigned csr_clock_hz)
{
	static gmac_hal_context_t ctx;
	const uint16_t test_len = 512U;
	uint8_t *pkt;
	gmac_speed_t speed;

#if (GMAC_PHY_INTF == EQOS_PHY_INTF_RGMII)
	speed = GMAC_SPEED_1000M;
#else
	speed = GMAC_SPEED_100M;
#endif

	GMAC_PRINTF("\n======== transmit and receive longtime test(force_link_phy) 开始 ========\n");
	s_dma = gmac_eqos_new_dma();
	pkt = (uint8_t *)s_dma->tx_buf[0];
	eqos_prepare_broadcast_test_packet(pkt, test_len);

	if (eqos_setup_force_link_phy_near_loopback(&ctx, csr_base, csr_clock_hz, speed))
		return;

	while (1) {
		int timeout = 200;

		(void)gmac_mac_transmit(&ctx, pkt, test_len);
		while (timeout--) {
			if (gmac_get_receive_finish_int_flag(&ctx) &&
			    gmac_receive_frame(&ctx, NULL, true, true, false)) {
				GMAC_PRINTF("longtime pass len=%u\n", test_len);
				break;
			}
			gmac_board_delay_us(20000U);
		}
		if (timeout <= 0)
			GMAC_PRINTF("longtime receive timeout\n");
	}
}
