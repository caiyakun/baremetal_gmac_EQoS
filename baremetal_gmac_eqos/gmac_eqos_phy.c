/*
 * PHY 层:Microchip VSC8541 + Synopsys DWMAC4/GMAC4 内建 MDIO(IEEE 802.3 Clause 22)
 *
 * 参考:
 *   - P5 工程 .../test_gmac.c 中 PHY 初始化与 force link 流程
 *   - Linux stmmac 对 GMAC_MDIO_ADDR 的打包格式(drivers/net/ethernet/stmicro/stmmac/stmmac_mdio.c)
 *
 * MDIO 事务要点(GMAC_MDIO_ADDR,与 Linux stmmac_mdio_read/write 一致):
 *   - bit0  BUSY:写 ADDR 触发读写后硬件置位,完成后清零；软件须等待 idle 再发起下一次。
 *   - bit3:2 GOC:读=3<<2,写=1<<2(GMAC4 模式,区别于旧 DWC 的 separate CR 寄存器)。
 *   - bit25:21 PA:PHY 地址 0~31；bit20:16 RDA:Clause 22 寄存器号 0~31。
 *   - 写流程:先写 DATA 低 16 位,再写 ADDR(含 BUSY|GOC|PA|RDA)；读流程:写 ADDR 后读 DATA。
 *   - CSR[11:8](EQOS_MDIO_CLK_CSR_SHIFT):MDC 相对 CSR 时钟分频,缺省 5,可按 SoC 要求用
 *     eqos_mdio_csr_apply() 或编译选项覆盖。
 *
 * 调试:所有用户可见日志经 GMAC_PRINTF(可在包含本模块前 #define GMAC_PRINTF 重定向到串口)。
 */
#include "gmac_eqos_hal.h"
#include "eqos_regs.h"

#ifndef GMAC_PRINTF
#include <stdio.h>
#define GMAC_PRINTF(...) printf(__VA_ARGS__)
#endif

/* vcs8541_init 内:条件为真表示失败 */
#define PHY_INIT_CHECK(cond, step)                                              \
	do {                                                                    \
		if (cond) {                                                     \
			GMAC_PRINTF("EQoS PHY: vcs8541_init 失败 @ %s\n", step); \
			return;                                                 \
		}                                                               \
	} while (0)

#define EQOS_BIT(n)		(1U << (n))
#define ETH_PHY_BMCR_REG_ADDR	0x00U
#define ETH_PHY_BMSR_REG_ADDR	0x01U
#define ETH_PHY_IDR1_REG_ADDR	0x02U
#define ETH_PHY_IDR2_REG_ADDR	0x03U
#define ETH_PHY_ANAR_REG_ADDR	0x04U
#define ETH_PHY_ANLPAR_REG_ADDR	0x05U
#define GMAC_PHY_PCR_REG_ADDR	0x1FU
#define GMAC_PHY_EPC_REG_ADDR	0x17U
#define DACS_PHY_DACS_REG_ADDR	0x1CU
#define GMAC_PHY_RCR_REG_ADDR	0x14U
#define GMAC_PHY_GC2R_REG_ADDR	0x0EU

#define MII_ADDR_BUSY		1U
#define MII_GMAC4_GOC_SHIFT	2U
#define MII_GMAC4_READ		(3U << MII_GMAC4_GOC_SHIFT)
#define MII_GMAC4_WRITE		(1U << MII_GMAC4_GOC_SHIFT)
#define MII_DATA_MASK		0xffffU

#ifndef EQOS_MDIO_CLK_CSR_SHIFT
#define EQOS_MDIO_CLK_CSR_SHIFT	8U
#endif
#ifndef EQOS_MDIO_CLK_CSR_DEFAULT
#define EQOS_MDIO_CLK_CSR_DEFAULT 5U
#endif

/*
 * 以下为 VSC8541 扩展寄存器在「位域视图」下的联合体,便于与手册位名对应；
 * 实际 MDIO 仍按 16 位寄存器读写。
 */
typedef union {
	struct {
		uint32_t register_page_select : 16;
	};
	uint32_t val;
} gmc_pcr_reg_t;

typedef union {
	struct {
		uint32_t reserved : 3;
		uint32_t far_end_loopback : 1;
		uint32_t reserved1 : 7;
		uint32_t mac_interface_sel : 2;
		uint32_t mac_supplied_clk_en : 1;
		uint32_t reserved2 : 2;
	};
	uint32_t val;
} gmac_epc_reg_t;

typedef union {
	struct {
		uint32_t tx_clk_delay : 3;
		uint32_t rgmii_txd_reversal : 1;
		uint32_t rx_clk_delay : 3;
		uint32_t rgmii_rxd_reversal : 1;
		uint32_t reserved1 : 4;
		uint32_t sof_en : 1;
		uint32_t reserved2 : 2;
		uint32_t flf2_en : 1;
	};
	uint32_t val;
} gmac_rcr_reg_t;

typedef union {
	struct {
		uint32_t reserved1 : 9;
		uint32_t tri_state_en : 1;
		uint32_t reserved2 : 1;
		uint32_t coma_mode_input_pin_data : 1;
		uint32_t coma_mode_output_pin_data : 1;
		uint32_t coma_mode_outout_en : 1;
		uint32_t reserved3 : 2;
	};
	uint32_t val;
} gmac_gc2r_reg_t;

typedef union {
	struct {
		uint32_t reserved : 7;
		uint32_t collision_test : 1;
		uint32_t duplex_mode : 1;
		uint32_t restart_auto_nego : 1;
		uint32_t isolate : 1;
		uint32_t power_down : 1;
		uint32_t en_auto_nego : 1;
		uint32_t speed_select : 1;
		uint32_t en_loopback : 1;
		uint32_t reset : 1;
	};
	uint32_t val;
} bmcr_reg_t;

phy_config_t g_eqos_phy_config = {
	.addr = GMAC_PHY_ADDR_AUTO,
	.reset_timeout_ms = 500U,
	.autonego_timeout_ms = 3000U,
	.link_status = GMAC_LINK_DOWN,
	.reset_gpio_num = -1,
	.hw_reset_assert_time_us = 10000U,
	.post_hw_reset_delay_ms = 10U,
};

static struct {
	uint32_t emacx_tx_clk_delay;
	uint32_t emacx_rx_clk_delay;
} s_rgmii_delay = { 0x05U, 0x05U };

/*
 * 板级 weak 钩子(建议在真实 BSP 中提供强符号):
 *   gmac_board_delay_us        — 微秒级阻塞延时(无 OS 时常用空转；精度影响 MDIO 超时判断)。
 *   gmac_board_emac_clock_and_pad_init — EMAC/PHY 参考时钟、IO 复用、RGMII 摆幅等。
 *   gmac_board_phy_gpio_reset  — 若 phy_config_t.reset_gpio_num >= 0,在 glb 与 phy_reset_hw 中调用。
 */
__attribute__((weak)) void gmac_board_delay_us(unsigned us)
{
	volatile unsigned n = us * 10U;

	while (n--)
		;
}

__attribute__((weak)) void gmac_board_emac_clock_and_pad_init(gmac_hal_context_t *hal)
{
	(void)hal;
}

__attribute__((weak)) void gmac_board_phy_gpio_reset(int gpio_num, int assert_level_us,
						   int release_unused)
{
	(void)gpio_num;
	(void)assert_level_us;
	(void)release_unused;
}

static uint32_t phy_reg_rd(gmac_hal_context_t *h, uint32_t off)
{
	return gmac_io_read32(h->csr_base + (uintptr_t)off);
}

static void phy_reg_wr(gmac_hal_context_t *h, uint32_t off, uint32_t v)
{
	gmac_io_write32(h->csr_base + (uintptr_t)off, v);
}

static int eqos_mdio_wait_idle(gmac_hal_context_t *h)
{
	unsigned to;

	for (to = 0; to < 10000U; to++) {
		if (!(phy_reg_rd(h, GMAC_MDIO_ADDR) & MII_ADDR_BUSY))
			return 0;
		gmac_board_delay_us(1);
	}
	GMAC_PRINTF("EQoS MDIO: 超时 — GMAC_MDIO_ADDR BUSY 未在 %u 次轮询内清除(检查 MDC/MDIO 与时钟)\n",
		    10000U);
	return -1;
}

static void eqos_mdio_csr_apply(gmac_hal_context_t *h, unsigned csr_field)
{
	uint32_t v;

	if (eqos_mdio_wait_idle(h)) {
		GMAC_PRINTF("EQoS MDIO: 无法应用 CSR 分频(MDIO 忙)\n");
		return;
	}
	v = phy_reg_rd(h, GMAC_MDIO_ADDR);
	v &= ~(0xfU << EQOS_MDIO_CLK_CSR_SHIFT);
	v |= (csr_field & 0xfU) << EQOS_MDIO_CLK_CSR_SHIFT;
	phy_reg_wr(h, GMAC_MDIO_ADDR, v);
	GMAC_PRINTF("EQoS MDIO: CSR 分频域已设为 %u(原 ADDR 部分位保留)\n",
		    (unsigned)(csr_field & 0xfU));
}

static uint32_t mdio_fmt_addr(unsigned pa, unsigned reg)
{
	return (((pa & 0x1fU) << 21) | ((reg & 0x1fU) << 16) | MII_ADDR_BUSY);
}

int gmac_write_phy_reg(gmac_hal_context_t *hal, uint32_t phy_addr, uint32_t phy_reg,
		       uint32_t reg_value)
{
	uint32_t addr;

	if (eqos_mdio_wait_idle(hal)) {
		GMAC_PRINTF("EQoS MDIO: 写 PHY%u reg%02u=0x%04lx 前 wait_idle 失败\n",
			    (unsigned)phy_addr, (unsigned)phy_reg,
			    (unsigned long)(reg_value & MII_DATA_MASK));
		return -1;
	}
	phy_reg_wr(hal, GMAC_MDIO_DATA, reg_value & MII_DATA_MASK);
	addr = mdio_fmt_addr(phy_addr, phy_reg) | MII_GMAC4_WRITE;
	phy_reg_wr(hal, GMAC_MDIO_ADDR, addr);
	if (eqos_mdio_wait_idle(hal)) {
		GMAC_PRINTF("EQoS MDIO: 写 PHY%u reg%02u=0x%04lx 后完成等待失败\n",
			    (unsigned)phy_addr, (unsigned)phy_reg,
			    (unsigned long)(reg_value & MII_DATA_MASK));
		return -1;
	}
	return 0;
}

int gmac_read_phy_reg(gmac_hal_context_t *hal, uint32_t phy_addr, uint32_t phy_reg,
		      uint32_t *reg_value)
{
	uint32_t addr;

	if (eqos_mdio_wait_idle(hal)) {
		GMAC_PRINTF("EQoS MDIO: 读 PHY%u reg%02u 前 wait_idle 失败\n",
			    (unsigned)phy_addr, (unsigned)phy_reg);
		return -1;
	}
	phy_reg_wr(hal, GMAC_MDIO_DATA, 0);
	addr = mdio_fmt_addr(phy_addr, phy_reg) | MII_GMAC4_READ;
	phy_reg_wr(hal, GMAC_MDIO_ADDR, addr);
	if (eqos_mdio_wait_idle(hal)) {
		GMAC_PRINTF("EQoS MDIO: 读 PHY%u reg%02u 后完成等待失败\n",
			    (unsigned)phy_addr, (unsigned)phy_reg);
		return -1;
	}
	*reg_value = phy_reg_rd(hal, GMAC_MDIO_DATA) & MII_DATA_MASK;
	return 0;
}

static int phy_set_reg_bits(gmac_hal_context_t *hal, uint32_t phy_addr, uint32_t reg_addr,
			    uint16_t and_mask, uint16_t or_mask)
{
	uint32_t v;

	if (gmac_read_phy_reg(hal, phy_addr, reg_addr, &v))
		return -1;
	v = (v & (uint32_t)and_mask) | (uint32_t)or_mask;
	return gmac_write_phy_reg(hal, phy_addr, reg_addr, v);
}

/* 通过读 Clause22 reg2(IDR1)判断该 MDIO 地址是否有器件响应 */
static int phy_detect_addr(gmac_hal_context_t *hal, int *detected)
{
	int a;
	uint32_t v;

	for (a = 0; a < 32; a++) {
		if (gmac_read_phy_reg(hal, (uint32_t)a, ETH_PHY_IDR1_REG_ADDR, &v))
			continue;
		if (v != 0xffffU && v != 0U) {
			*detected = a;
			GMAC_PRINTF("EQoS PHY: found PHY at MDIO addr %d (IDR1=0x%lx)\n", a,
				    (unsigned long)v);
			return 0;
		}
	}
	GMAC_PRINTF("EQoS PHY: no PHY detected on MDIO\n");
	return -1;
}

static int phy_page_select(phy_config_t *pc, gmac_hal_context_t *hal, uint32_t page)
{
	gmc_pcr_reg_t pcr = { .register_page_select = page & 0xffffU };

	return gmac_write_phy_reg(hal, (uint32_t)pc->addr, GMAC_PHY_PCR_REG_ADDR, pcr.val);
}

static int phy_802_3_pwrctl(phy_config_t *pc, gmac_hal_context_t *hal, bool enable)
{
	bmcr_reg_t bmcr;

	GMAC_PRINTF("EQoS PHY: BMCR 电源 — %s(PHY 地址 %d)\n",
		    enable ? "上电/退出掉电" : "掉电", pc->addr);
	if (gmac_read_phy_reg(hal, (uint32_t)pc->addr, ETH_PHY_BMCR_REG_ADDR, &bmcr.val))
		return -1;
	if (!enable)
		bmcr.power_down = 1;
	else
		bmcr.power_down = 0;
	if (gmac_write_phy_reg(hal, (uint32_t)pc->addr, ETH_PHY_BMCR_REG_ADDR, bmcr.val))
		return -1;
	if (enable) {
		unsigned to;

		for (to = 0; to < pc->reset_timeout_ms / 10U; to++) {
			gmac_board_delay_us(10000U);
			if (gmac_read_phy_reg(hal, (uint32_t)pc->addr, ETH_PHY_BMCR_REG_ADDR,
					       &bmcr.val))
				return -1;
			if (!bmcr.power_down)
				break;
		}
	}
	return 0;
}

static int phy_802_3_reset(phy_config_t *pc, gmac_hal_context_t *hal)
{
	bmcr_reg_t bmcr = { .reset = 1 };
	unsigned to;

	GMAC_PRINTF("EQoS PHY: 发起 IEEE802.3 软件复位(BMCR.reset)\n");
	if (gmac_write_phy_reg(hal, (uint32_t)pc->addr, ETH_PHY_BMCR_REG_ADDR, bmcr.val))
		return -1;
	for (to = 0; to < pc->reset_timeout_ms / 10U; to++) {
		gmac_board_delay_us(10000U);
		if (gmac_read_phy_reg(hal, (uint32_t)pc->addr, ETH_PHY_BMCR_REG_ADDR, &bmcr.val))
			return -1;
		if (!bmcr.reset) {
			GMAC_PRINTF("EQoS PHY: software reset complete\n");
			break;
		}
	}
	return 0;
}

int gmac_phy_reset_hw(phy_config_t *phy_config_info)
{
	if (phy_config_info->reset_gpio_num < 0) {
		GMAC_PRINTF("EQoS PHY: 跳过 GPIO 硬件复位(reset_gpio_num=%d)\n",
			    phy_config_info->reset_gpio_num);
		return 0;
	}
	GMAC_PRINTF("EQoS PHY: GPIO 硬件复位 nRST gpio=%d 保持约 %u us,释放后延时 %u ms\n",
		    phy_config_info->reset_gpio_num,
		    phy_config_info->hw_reset_assert_time_us,
		    phy_config_info->post_hw_reset_delay_ms);
	gmac_board_phy_gpio_reset(phy_config_info->reset_gpio_num,
				  (int)phy_config_info->hw_reset_assert_time_us, 0);
	gmac_board_delay_us(phy_config_info->post_hw_reset_delay_ms * 1000U);
	return 0;
}

void gmac_glb_cfg_init(gmac_hal_context_t *hal)
{
	GMAC_PRINTF("EQoS glb: 全局初始化 csr_base=%p\n", (void *)hal->csr_base);
	GMAC_PRINTF("EQoS glb: 调用板级 EMAC 时钟与 PAD(gmac_board_emac_clock_and_pad_init)\n");
	gmac_board_emac_clock_and_pad_init(hal);
	GMAC_PRINTF("EQoS glb: 配置 MDIO MDC 分频(eqos_mdio_csr_apply)\n");
	eqos_mdio_csr_apply(hal, EQOS_MDIO_CLK_CSR_DEFAULT);
	gmac_phy_reset_hw(&g_eqos_phy_config);
	gmac_board_delay_us(g_eqos_phy_config.post_hw_reset_delay_ms * 1000U);
	GMAC_PRINTF("EQoS glb: 全局初始化完成\n");
}

/* 上电清除 power_down 后对 BMCR 置 reset,等待 PHY 自清 */
static int phy_802_3_basic_phy_init(phy_config_t *pc, gmac_hal_context_t *hal)
{
	GMAC_PRINTF("EQoS PHY: phy_802_3_basic_phy_init — 上电 + 软复位\n");
	if (phy_802_3_pwrctl(pc, hal, true))
		return -1;
	return phy_802_3_reset(pc, hal);
}

void gmac_phy_802_3_basic_phy_deinit(phy_config_t *phy_cfg, gmac_hal_context_t *hal)
{
	GMAC_PRINTF("EQoS PHY: deinit — 清除 link 状态并 BMCR 掉电(PHY 地址 %d)\n",
		    phy_cfg->addr);
	phy_cfg->link_status = GMAC_LINK_DOWN;
	if (phy_802_3_pwrctl(phy_cfg, hal, false))
		GMAC_PRINTF("EQoS PHY: deinit 掉电写 BMCR 失败\n");
}

/*
 * VSC8541 上电后典型顺序:选页 0 → EPC 选 MAC 侧接口 → 802.3 上电+软复位 →(RGMII)页 2 调 RCR 延时
 * → 页 0 配 ANAR/1000BASE-T 能力广播 → 可选 phy_extra 强制速率/双工/环回 → 扩展页写 GC2R(coma 相关)。
 * phy_extra 为 NULL 时仅做能力广播与接口配置,不强制 BMCR 链路参数。
 */
void gmac_phy_vcs8541_init(phy_config_t *phy_cfg, gmac_hal_context_t *hal,
			    phy_extra_config_t *p_phy_extra_conf)
{
	uint32_t addr;
	gmac_epc_reg_t ext_phy_ctrl_val;
	gmac_rcr_reg_t rgmii_ctrl_reg_val;
	gmac_gc2r_reg_t gpio_ctrl2_reg_val;
	uint32_t id1 = 0;
	uint32_t id2 = 0;

	GMAC_PRINTF("EQoS PHY: ---------- gmac_phy_vcs8541_init 开始 ----------\n");

	if (!phy_cfg)
		phy_cfg = &g_eqos_phy_config;

	if (phy_cfg->addr == GMAC_PHY_ADDR_AUTO) {
		int d;

		GMAC_PRINTF("EQoS PHY: PHY 地址为 AUTO,在 MDIO 0~31 上扫描 IDR1…\n");
		if (phy_detect_addr(hal, &d)) {
			GMAC_PRINTF("EQoS PHY: 自动扫描未找到 PHY,终止初始化\n");
			return;
		}
		phy_cfg->addr = d;
	}
	addr = (uint32_t)phy_cfg->addr;
	phy_cfg->link_status = GMAC_LINK_DOWN;
	GMAC_PRINTF("EQoS PHY: 使用 MDIO 地址 %u,link_status 先置 DOWN\n", (unsigned)addr);

	PHY_INIT_CHECK(gmac_read_phy_reg(hal, addr, ETH_PHY_IDR1_REG_ADDR, &id1) != 0,
		       "读 IDR1(Clause 22 reg2)");
	PHY_INIT_CHECK(gmac_read_phy_reg(hal, addr, ETH_PHY_IDR2_REG_ADDR, &id2) != 0,
		       "读 IDR2(Clause 22 reg3)");
	GMAC_PRINTF("EQoS PHY: PHY ID — IDR1=0x%04lx IDR2=0x%04lx(可对照 OUI/型号)\n",
		    (unsigned long)id1, (unsigned long)id2);

	GMAC_PRINTF("EQoS PHY: 选扩展页 0,读/写 EPC(reg 0x17)设置 MAC-PHY 接口类型\n");
	PHY_INIT_CHECK(phy_page_select(phy_cfg, hal, 0x00U) != 0, "PCR 选页 0");
	PHY_INIT_CHECK(gmac_read_phy_reg(hal, addr, GMAC_PHY_EPC_REG_ADDR, &ext_phy_ctrl_val.val) != 0,
		       "读 EPC");
#if GMAC_PHY_INTF == EQOS_PHY_INTF_RGMII
	ext_phy_ctrl_val.mac_interface_sel = 2U;
	GMAC_PRINTF("EQoS PHY: EPC.mac_interface_sel=RGMII(2)\n");
#elif GMAC_PHY_INTF == EQOS_PHY_INTF_MII_GMII
	ext_phy_ctrl_val.mac_interface_sel = 0U;
	GMAC_PRINTF("EQoS PHY: EPC.mac_interface_sel=MII/GMII(0)\n");
#else
	ext_phy_ctrl_val.mac_interface_sel = 1U;
	GMAC_PRINTF("EQoS PHY: EPC.mac_interface_sel=RMII(1)\n");
#endif
	PHY_INIT_CHECK(gmac_write_phy_reg(hal, addr, GMAC_PHY_EPC_REG_ADDR, ext_phy_ctrl_val.val) != 0,
		       "写 EPC");

	GMAC_PRINTF("EQoS PHY: IEEE802.3 基础上电 + BMCR 软件复位\n");
	PHY_INIT_CHECK(phy_802_3_basic_phy_init(phy_cfg, hal) != 0, "phy_802_3_basic_phy_init");

#if GMAC_PHY_INTF == EQOS_PHY_INTF_RGMII
	GMAC_PRINTF("EQoS PHY: RGMII — 扩展页 2 写 RCR:TX/RX 时钟延时各 3bit(当前 tx=%u rx=%u)\n",
		    (unsigned)(s_rgmii_delay.emacx_tx_clk_delay & 7U),
		    (unsigned)(s_rgmii_delay.emacx_rx_clk_delay & 7U));
	PHY_INIT_CHECK(phy_page_select(phy_cfg, hal, 0x02U) != 0, "PCR 选页 2(RGMII 控制)");
	PHY_INIT_CHECK(gmac_read_phy_reg(hal, addr, GMAC_PHY_RCR_REG_ADDR, &rgmii_ctrl_reg_val.val) != 0,
		       "读 RCR");
	rgmii_ctrl_reg_val.val &= ~(1U << 11);
	rgmii_ctrl_reg_val.rx_clk_delay = s_rgmii_delay.emacx_rx_clk_delay & 7U;
	rgmii_ctrl_reg_val.tx_clk_delay = s_rgmii_delay.emacx_tx_clk_delay & 7U;
	PHY_INIT_CHECK(gmac_write_phy_reg(hal, addr, GMAC_PHY_RCR_REG_ADDR, rgmii_ctrl_reg_val.val) != 0,
		       "写 RCR");
	GMAC_PRINTF("EQoS PHY: RCR 已写 raw=0x%04lx\n", (unsigned long)rgmii_ctrl_reg_val.val);
#endif

	GMAC_PRINTF("EQoS PHY: 回到页 0,配置 ANAR 暂停帧位与自协商广播能力\n");
	PHY_INIT_CHECK(phy_page_select(phy_cfg, hal, 0x00U) != 0, "PCR 回到页 0");

	(void)phy_set_reg_bits(hal, addr, ETH_PHY_ANAR_REG_ADDR,
			       (uint16_t)~(EQOS_BIT(10)), (uint16_t)EQOS_BIT(10));
	(void)phy_set_reg_bits(hal, addr, ETH_PHY_ANAR_REG_ADDR,
			       (uint16_t)~(EQOS_BIT(11)), 0);

#if GMAC_PHY_INTF == EQOS_PHY_INTF_RGMII
	if (VSC8541PHY_ADVERTISE_1000BASET_FDX)
		(void)phy_set_reg_bits(hal, addr, 0x09U, (uint16_t)~(EQOS_BIT(9)), (uint16_t)EQOS_BIT(9));
	else
		(void)phy_set_reg_bits(hal, addr, 0x09U, (uint16_t)~(EQOS_BIT(9)), 0);
	if (VSC8541PHY_ADVERTISE_1000BASET_HDX)
		(void)phy_set_reg_bits(hal, addr, 0x09U, (uint16_t)~(EQOS_BIT(8)), (uint16_t)EQOS_BIT(8));
	else
		(void)phy_set_reg_bits(hal, addr, 0x09U, (uint16_t)~(EQOS_BIT(8)), 0);
#else
	(void)phy_set_reg_bits(hal, addr, 0x09U, (uint16_t)~(EQOS_BIT(9)), 0);
	(void)phy_set_reg_bits(hal, addr, 0x09U, (uint16_t)~(EQOS_BIT(8)), 0);
#endif

	if (VSC8541PHY_ADVERTISE_100BASETX_FDX)
		(void)phy_set_reg_bits(hal, addr, ETH_PHY_ANAR_REG_ADDR,
				       (uint16_t)~(EQOS_BIT(8)), (uint16_t)EQOS_BIT(8));
	else
		(void)phy_set_reg_bits(hal, addr, ETH_PHY_ANAR_REG_ADDR,
				       (uint16_t)~(EQOS_BIT(8)), 0);
	if (VSC8541PHY_ADVERTISE_100BASETX_HDX)
		(void)phy_set_reg_bits(hal, addr, ETH_PHY_ANAR_REG_ADDR,
				       (uint16_t)~(EQOS_BIT(7)), (uint16_t)EQOS_BIT(7));
	else
		(void)phy_set_reg_bits(hal, addr, ETH_PHY_ANAR_REG_ADDR,
				       (uint16_t)~(EQOS_BIT(7)), 0);
	if (VSC8541PHY_ADVERTISE_10BASET_FDX)
		(void)phy_set_reg_bits(hal, addr, ETH_PHY_ANAR_REG_ADDR,
				       (uint16_t)~(EQOS_BIT(6)), (uint16_t)EQOS_BIT(6));
	else
		(void)phy_set_reg_bits(hal, addr, ETH_PHY_ANAR_REG_ADDR,
				       (uint16_t)~(EQOS_BIT(6)), 0);
	if (VSC8541PHY_ADVERTISE_10BASET_HDX)
		(void)phy_set_reg_bits(hal, addr, ETH_PHY_ANAR_REG_ADDR,
				       (uint16_t)~(EQOS_BIT(5)), (uint16_t)EQOS_BIT(5));
	else
		(void)phy_set_reg_bits(hal, addr, ETH_PHY_ANAR_REG_ADDR,
				       (uint16_t)~(EQOS_BIT(5)), 0);
	GMAC_PRINTF("EQoS PHY: ANAR/1000BASE-T 能力位已按编译期宏刷写(失败未单独报错)\n");

	if (p_phy_extra_conf) {
		GMAC_PRINTF("EQoS PHY: phy_extra — force_speed=%d force_duplex=%d "
			    "phy_loopback=%d near_end_lb=%d\n",
			    (int)p_phy_extra_conf->force_link_speed,
			    (int)p_phy_extra_conf->force_duplex,
			    (int)p_phy_extra_conf->phy_loopback_en,
			    (int)p_phy_extra_conf->is_phy_near_end_loopback);
		(void)phy_set_reg_bits(hal, addr, ETH_PHY_BMCR_REG_ADDR,
				       (uint16_t)~(EQOS_BIT(12)), 0);
		GMAC_PRINTF("EQoS PHY: 已清 BMCR.an_enable(bit12),进入强制链路参数路径\n");

		if (p_phy_extra_conf->force_link_speed == GMAC_SPEED_100M) {
			GMAC_PRINTF("EQoS PHY: 强制 100M:BMCR speed_select + 100M 选择\n");
			(void)phy_set_reg_bits(hal, addr, ETH_PHY_BMCR_REG_ADDR,
					       (uint16_t)~(EQOS_BIT(8)), (uint16_t)EQOS_BIT(8));
			(void)phy_set_reg_bits(hal, addr, ETH_PHY_BMCR_REG_ADDR,
					       (uint16_t)~(EQOS_BIT(6) | EQOS_BIT(13)),
					       (uint16_t)EQOS_BIT(13));
		} else if (p_phy_extra_conf->force_link_speed == GMAC_SPEED_10M) {
			GMAC_PRINTF("EQoS PHY: 强制 10M\n");
			(void)phy_set_reg_bits(hal, addr, ETH_PHY_BMCR_REG_ADDR,
					       (uint16_t)~(EQOS_BIT(8)), (uint16_t)EQOS_BIT(8));
			(void)phy_set_reg_bits(hal, addr, ETH_PHY_BMCR_REG_ADDR,
					       (uint16_t)~(EQOS_BIT(6) | EQOS_BIT(13)), 0);
		} else if (p_phy_extra_conf->force_link_speed == GMAC_SPEED_1000M) {
			GMAC_PRINTF("EQoS PHY: 强制 1000BASE-T:写 reg9 + 扩展页 1 reg0x13 + BMCR\n");
			(void)phy_set_reg_bits(hal, addr, 0x09U, (uint16_t)~(EQOS_BIT(12)),
					       (uint16_t)EQOS_BIT(12));
			(void)phy_set_reg_bits(hal, addr, 0x09U, (uint16_t)~(EQOS_BIT(11)),
					       (uint16_t)EQOS_BIT(11));
			PHY_INIT_CHECK(phy_page_select(phy_cfg, hal, 0x01U) != 0,
				       "1000M:PCR 选页 1");
			(void)phy_set_reg_bits(hal, addr, 0x13U,
					       (uint16_t)~(EQOS_BIT(3) | EQOS_BIT(2)),
					       (uint16_t)EQOS_BIT(3));
			PHY_INIT_CHECK(phy_page_select(phy_cfg, hal, 0x00U) != 0,
				       "1000M:PCR 回页 0");
			(void)phy_set_reg_bits(hal, addr, ETH_PHY_BMCR_REG_ADDR,
					       (uint16_t)~(EQOS_BIT(8)), (uint16_t)EQOS_BIT(8));
			(void)phy_set_reg_bits(hal, addr, ETH_PHY_BMCR_REG_ADDR,
					       (uint16_t)~(EQOS_BIT(6) | EQOS_BIT(13)),
					       (uint16_t)EQOS_BIT(6));
		} else {
			GMAC_PRINTF("EQoS PHY: 警告 — force_link_speed 枚举值未识别,跳过速率强制\n");
		}

		if (p_phy_extra_conf->force_duplex == GMAC_DUPLEX_FULL) {
			GMAC_PRINTF("EQoS PHY: BMCR 全双工(bit8)\n");
			(void)phy_set_reg_bits(hal, addr, ETH_PHY_BMCR_REG_ADDR,
					       (uint16_t)~(EQOS_BIT(8)), (uint16_t)EQOS_BIT(8));
		} else {
			GMAC_PRINTF("EQoS PHY: BMCR 半双工\n");
			(void)phy_set_reg_bits(hal, addr, ETH_PHY_BMCR_REG_ADDR,
					       (uint16_t)~(EQOS_BIT(8)), 0);
		}

		if (p_phy_extra_conf->phy_loopback_en) {
			if (p_phy_extra_conf->is_phy_near_end_loopback) {
				GMAC_PRINTF("EQoS PHY: PHY 近端环回 — BMCR.loopback bit14\n");
				(void)phy_set_reg_bits(hal, addr, ETH_PHY_BMCR_REG_ADDR,
						       (uint16_t)~(EQOS_BIT(14)),
						       (uint16_t)EQOS_BIT(14));
			} else {
				GMAC_PRINTF("EQoS PHY: PHY 远端环回 — EPC 相关位(reg0x17 bit3)\n");
				(void)phy_set_reg_bits(hal, addr, 0x17U, (uint16_t)~(EQOS_BIT(3)),
						       (uint16_t)EQOS_BIT(3));
			}
		}
	} else {
		GMAC_PRINTF("EQoS PHY: phy_extra 为 NULL,跳过强制速率/双工/PHY 环回\n");
	}

	GMAC_PRINTF("EQoS PHY: 扩展页 0x10 写 GC2R=0x200(coma_mode 相关,对齐 P5 流程)\n");
	PHY_INIT_CHECK(phy_page_select(phy_cfg, hal, 0x10U) != 0, "PCR 选页 0x10");
	gpio_ctrl2_reg_val.val = 0x200U;
	PHY_INIT_CHECK(gmac_write_phy_reg(hal, addr, GMAC_PHY_GC2R_REG_ADDR, gpio_ctrl2_reg_val.val) != 0,
		       "写 GC2R");
	(void)phy_page_select(phy_cfg, hal, 0x00U);

	GMAC_PRINTF("EQoS PHY: ---------- gmac_phy_vcs8541_init 完成 (MDIO addr=%u) ----------\n",
		    (unsigned)addr);
}
