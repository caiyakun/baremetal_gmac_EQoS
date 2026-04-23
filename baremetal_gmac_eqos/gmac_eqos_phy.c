/*
 * PHY：VSC8541 + Synopsys EQoS MDIO（Clause 22），流程对齐
 * P5 .../test_gmac.c 与 esp_eth_phy_vcs8541.c
 */
#include "gmac_eqos_hal.h"
#include "eqos_regs.h"

#ifndef GMAC_PRINTF
#include <stdio.h>
#define GMAC_PRINTF(...) printf(__VA_ARGS__)
#endif

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
	return -1;
}

static void eqos_mdio_csr_apply(gmac_hal_context_t *h, unsigned csr_field)
{
	uint32_t v;

	if (eqos_mdio_wait_idle(h))
		return;
	v = phy_reg_rd(h, GMAC_MDIO_ADDR);
	v &= ~(0xfU << EQOS_MDIO_CLK_CSR_SHIFT);
	v |= (csr_field & 0xfU) << EQOS_MDIO_CLK_CSR_SHIFT;
	phy_reg_wr(h, GMAC_MDIO_ADDR, v);
}

static uint32_t mdio_fmt_addr(unsigned pa, unsigned reg)
{
	return (((pa & 0x1fU) << 21) | ((reg & 0x1fU) << 16) | MII_ADDR_BUSY);
}

int gmac_write_phy_reg(gmac_hal_context_t *hal, uint32_t phy_addr, uint32_t phy_reg,
		       uint32_t reg_value)
{
	uint32_t addr;

	if (eqos_mdio_wait_idle(hal))
		return -1;
	phy_reg_wr(hal, GMAC_MDIO_DATA, reg_value & MII_DATA_MASK);
	addr = mdio_fmt_addr(phy_addr, phy_reg) | MII_GMAC4_WRITE;
	phy_reg_wr(hal, GMAC_MDIO_ADDR, addr);
	return eqos_mdio_wait_idle(hal);
}

int gmac_read_phy_reg(gmac_hal_context_t *hal, uint32_t phy_addr, uint32_t phy_reg,
		      uint32_t *reg_value)
{
	uint32_t addr;

	if (eqos_mdio_wait_idle(hal))
		return -1;
	phy_reg_wr(hal, GMAC_MDIO_DATA, 0);
	addr = mdio_fmt_addr(phy_addr, phy_reg) | MII_GMAC4_READ;
	phy_reg_wr(hal, GMAC_MDIO_ADDR, addr);
	if (eqos_mdio_wait_idle(hal))
		return -1;
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
	if (phy_config_info->reset_gpio_num < 0)
		return 0;
	gmac_board_phy_gpio_reset(phy_config_info->reset_gpio_num,
				  (int)phy_config_info->hw_reset_assert_time_us, 0);
	gmac_board_delay_us(phy_config_info->post_hw_reset_delay_ms * 1000U);
	return 0;
}

void gmac_glb_cfg_init(gmac_hal_context_t *hal)
{
	GMAC_PRINTF("EQoS glb: csr_base=%p\n", (void *)hal->csr_base);
	gmac_board_emac_clock_and_pad_init(hal);
	eqos_mdio_csr_apply(hal, EQOS_MDIO_CLK_CSR_DEFAULT);
	gmac_phy_reset_hw(&g_eqos_phy_config);
	gmac_board_delay_us(g_eqos_phy_config.post_hw_reset_delay_ms * 1000U);
}

static int phy_802_3_basic_phy_init(phy_config_t *pc, gmac_hal_context_t *hal)
{
	if (phy_802_3_pwrctl(pc, hal, true))
		return -1;
	return phy_802_3_reset(pc, hal);
}

void gmac_phy_802_3_basic_phy_deinit(phy_config_t *phy_cfg, gmac_hal_context_t *hal)
{
	phy_cfg->link_status = GMAC_LINK_DOWN;
	(void)phy_802_3_pwrctl(phy_cfg, hal, false);
}

void gmac_phy_vcs8541_init(phy_config_t *phy_cfg, gmac_hal_context_t *hal,
			    phy_extra_config_t *p_phy_extra_conf)
{
	uint32_t addr;
	gmac_epc_reg_t ext_phy_ctrl_val;
	gmac_rcr_reg_t rgmii_ctrl_reg_val;
	gmac_gc2r_reg_t gpio_ctrl2_reg_val;
	uint32_t id1 = 0;
	uint32_t id2 = 0;

	if (!phy_cfg)
		phy_cfg = &g_eqos_phy_config;

	if (phy_cfg->addr == GMAC_PHY_ADDR_AUTO) {
		int d;

		if (phy_detect_addr(hal, &d))
			return;
		phy_cfg->addr = d;
	}
	addr = (uint32_t)phy_cfg->addr;
	phy_cfg->link_status = GMAC_LINK_DOWN;

	if (gmac_read_phy_reg(hal, addr, ETH_PHY_IDR1_REG_ADDR, &id1))
		return;
	if (gmac_read_phy_reg(hal, addr, ETH_PHY_IDR2_REG_ADDR, &id2))
		return;
	GMAC_PRINTF("EQoS PHY: IDR1=0x%lx IDR2=0x%lx\n", (unsigned long)id1,
		    (unsigned long)id2);

	(void)phy_page_select(phy_cfg, hal, 0x00U);
	if (gmac_read_phy_reg(hal, addr, GMAC_PHY_EPC_REG_ADDR, &ext_phy_ctrl_val.val))
		return;
#if GMAC_PHY_INTF == EQOS_PHY_INTF_RGMII
	ext_phy_ctrl_val.mac_interface_sel = 2U;
#elif GMAC_PHY_INTF == EQOS_PHY_INTF_MII_GMII
	ext_phy_ctrl_val.mac_interface_sel = 0U;
#else
	ext_phy_ctrl_val.mac_interface_sel = 1U;
#endif
	if (gmac_write_phy_reg(hal, addr, GMAC_PHY_EPC_REG_ADDR, ext_phy_ctrl_val.val))
		return;

	if (phy_802_3_basic_phy_init(phy_cfg, hal))
		return;

#if GMAC_PHY_INTF == EQOS_PHY_INTF_RGMII
	if (phy_page_select(phy_cfg, hal, 0x02U))
		return;
	if (gmac_read_phy_reg(hal, addr, GMAC_PHY_RCR_REG_ADDR, &rgmii_ctrl_reg_val.val))
		return;
	rgmii_ctrl_reg_val.val &= ~(1U << 11);
	rgmii_ctrl_reg_val.rx_clk_delay = s_rgmii_delay.emacx_rx_clk_delay & 7U;
	rgmii_ctrl_reg_val.tx_clk_delay = s_rgmii_delay.emacx_tx_clk_delay & 7U;
	if (gmac_write_phy_reg(hal, addr, GMAC_PHY_RCR_REG_ADDR, rgmii_ctrl_reg_val.val))
		return;
#endif

	if (phy_page_select(phy_cfg, hal, 0x00U))
		return;

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

	if (p_phy_extra_conf) {
		GMAC_PRINTF("EQoS PHY: force link speed=%d duplex=%d\n",
			    (int)p_phy_extra_conf->force_link_speed,
			    (int)p_phy_extra_conf->force_duplex);
		(void)phy_set_reg_bits(hal, addr, ETH_PHY_BMCR_REG_ADDR,
				       (uint16_t)~(EQOS_BIT(12)), 0);

		if (p_phy_extra_conf->force_link_speed == GMAC_SPEED_100M) {
			(void)phy_set_reg_bits(hal, addr, ETH_PHY_BMCR_REG_ADDR,
					       (uint16_t)~(EQOS_BIT(8)), (uint16_t)EQOS_BIT(8));
			(void)phy_set_reg_bits(hal, addr, ETH_PHY_BMCR_REG_ADDR,
					       (uint16_t)~(EQOS_BIT(6) | EQOS_BIT(13)),
					       (uint16_t)EQOS_BIT(13));
		} else if (p_phy_extra_conf->force_link_speed == GMAC_SPEED_10M) {
			(void)phy_set_reg_bits(hal, addr, ETH_PHY_BMCR_REG_ADDR,
					       (uint16_t)~(EQOS_BIT(8)), (uint16_t)EQOS_BIT(8));
			(void)phy_set_reg_bits(hal, addr, ETH_PHY_BMCR_REG_ADDR,
					       (uint16_t)~(EQOS_BIT(6) | EQOS_BIT(13)), 0);
		} else if (p_phy_extra_conf->force_link_speed == GMAC_SPEED_1000M) {
			(void)phy_set_reg_bits(hal, addr, 0x09U, (uint16_t)~(EQOS_BIT(12)),
					       (uint16_t)EQOS_BIT(12));
			(void)phy_set_reg_bits(hal, addr, 0x09U, (uint16_t)~(EQOS_BIT(11)),
					       (uint16_t)EQOS_BIT(11));
			if (phy_page_select(phy_cfg, hal, 0x01U))
				return;
			(void)phy_set_reg_bits(hal, addr, 0x13U,
					       (uint16_t)~(EQOS_BIT(3) | EQOS_BIT(2)),
					       (uint16_t)EQOS_BIT(3));
			if (phy_page_select(phy_cfg, hal, 0x00U))
				return;
			(void)phy_set_reg_bits(hal, addr, ETH_PHY_BMCR_REG_ADDR,
					       (uint16_t)~(EQOS_BIT(8)), (uint16_t)EQOS_BIT(8));
			(void)phy_set_reg_bits(hal, addr, ETH_PHY_BMCR_REG_ADDR,
					       (uint16_t)~(EQOS_BIT(6) | EQOS_BIT(13)),
					       (uint16_t)EQOS_BIT(6));
		}

		if (p_phy_extra_conf->force_duplex == GMAC_DUPLEX_FULL)
			(void)phy_set_reg_bits(hal, addr, ETH_PHY_BMCR_REG_ADDR,
					       (uint16_t)~(EQOS_BIT(8)), (uint16_t)EQOS_BIT(8));
		else
			(void)phy_set_reg_bits(hal, addr, ETH_PHY_BMCR_REG_ADDR,
					       (uint16_t)~(EQOS_BIT(8)), 0);

		if (p_phy_extra_conf->phy_loopback_en) {
			if (p_phy_extra_conf->is_phy_near_end_loopback)
				(void)phy_set_reg_bits(hal, addr, ETH_PHY_BMCR_REG_ADDR,
						       (uint16_t)~(EQOS_BIT(14)),
						       (uint16_t)EQOS_BIT(14));
			else
				(void)phy_set_reg_bits(hal, addr, 0x17U, (uint16_t)~(EQOS_BIT(3)),
						       (uint16_t)EQOS_BIT(3));
		}
	}

	if (phy_page_select(phy_cfg, hal, 0x10U))
		return;
	gpio_ctrl2_reg_val.val = 0x200U;
	if (gmac_write_phy_reg(hal, addr, GMAC_PHY_GC2R_REG_ADDR, gpio_ctrl2_reg_val.val))
		return;
	(void)phy_page_select(phy_cfg, hal, 0x00U);

	GMAC_PRINTF("EQoS PHY: vcs8541_init done (addr=%u)\n", (unsigned)addr);
}
