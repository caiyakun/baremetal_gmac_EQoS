/*
 * VSC8541 + IEEE802.3 Clause 22 PHY 配置（逻辑对齐 P5 test_gmac.c / esp_eth_phy_vcs8541.c）
 *
 * 包含关系：须通过 gmac_eqos_hal.h 间接包含（该头在定义 gmac_hal_context_t 后 include 本文件），
 * 以便使用 gmac_speed_t、gmac_duplex_t 与 gmac_hal_context_t。
 *
 * phy_config_t：MDIO 地址、复位超时、GPIO 复位脚（负数表示不用 GPIO）等。
 * phy_extra_config_t：可选「强制链路」参数，传给 gmac_phy_vcs8541_init；为 NULL 时不改 BMCR 速率/双工。
 * 日志与 gmac_eqos_hal 相同，由 GMAC_PRINTF 输出（见 gmac_eqos_hal.h 说明）。
 */
#ifndef GMAC_EQOS_PHY_H
#define GMAC_EQOS_PHY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifndef GMAC_PHY_ADDR_AUTO
#define GMAC_PHY_ADDR_AUTO	(-1)
#endif

#ifndef GMAC_LINK_DOWN
#define GMAC_LINK_DOWN		0
#endif
#ifndef GMAC_LINK_UP
#define GMAC_LINK_UP		1
#endif

/* 与 emac_ll.h 默认能力广播一致，可按项目重定义 */
#ifndef VSC8541PHY_ADVERTISE_1000BASET_FDX
#define VSC8541PHY_ADVERTISE_1000BASET_FDX	1
#endif
#ifndef VSC8541PHY_ADVERTISE_1000BASET_HDX
#define VSC8541PHY_ADVERTISE_1000BASET_HDX	0
#endif
#ifndef VSC8541PHY_ADVERTISE_100BASETX_FDX
#define VSC8541PHY_ADVERTISE_100BASETX_FDX	1
#endif
#ifndef VSC8541PHY_ADVERTISE_100BASETX_HDX
#define VSC8541PHY_ADVERTISE_100BASETX_HDX	1
#endif
#ifndef VSC8541PHY_ADVERTISE_10BASET_FDX
#define VSC8541PHY_ADVERTISE_10BASET_FDX	1
#endif
#ifndef VSC8541PHY_ADVERTISE_10BASET_HDX
#define VSC8541PHY_ADVERTISE_10BASET_HDX	1
#endif

/* PHY 与 MAC 侧介质：默认 RGMII，可在编译前定义 EQOS_PHY_INTF_RMII(1)/MII(2) 等扩展 */
#ifndef EQOS_PHY_INTF_RGMII
#define EQOS_PHY_INTF_RGMII	2
#endif
#ifndef EQOS_PHY_INTF_MII_GMII
#define EQOS_PHY_INTF_MII_GMII	0
#endif
#ifndef EQOS_PHY_INTF_RMII
#define EQOS_PHY_INTF_RMII	1
#endif
#ifndef GMAC_PHY_INTF
#define GMAC_PHY_INTF		EQOS_PHY_INTF_RGMII
#endif

typedef struct {
	int addr;			/* MDIO PHY 地址 0~31，或 GMAC_PHY_ADDR_AUTO 自动扫描 */
	unsigned reset_timeout_ms;	/* BMCR 软复位等轮询上限 */
	unsigned autonego_timeout_ms;	/* 预留：自协商超时 */
	int link_status;		/* GMAC_LINK_UP / GMAC_LINK_DOWN，由上层或链路检测更新 */
	int reset_gpio_num;		/* PHY nRST GPIO 编号，<0 表示不使用 GPIO 硬件复位 */
	unsigned hw_reset_assert_time_us;
	unsigned post_hw_reset_delay_ms;
} phy_config_t;

typedef struct {
	gmac_speed_t force_link_speed;	/* 与 MAC 侧 gmac_mac_set_speed 应对齐 */
	gmac_duplex_t force_duplex;
	bool phy_loopback_en;		/* true 时根据 is_phy_near_end_loopback 选近端/远端 PHY 环回 */
	bool is_phy_near_end_loopback;	/* true：BMCR loopback；false：EPC 远端环回位 */
} phy_extra_config_t;

extern phy_config_t g_eqos_phy_config;

void gmac_board_delay_us(unsigned us);
void gmac_board_emac_clock_and_pad_init(gmac_hal_context_t *hal);
void gmac_board_phy_gpio_reset(int gpio_num, int assert_level_us, int release);

int gmac_phy_reset_hw(phy_config_t *phy);
void gmac_glb_cfg_init(gmac_hal_context_t *hal);
void gmac_phy_vcs8541_init(phy_config_t *phy_cfg, gmac_hal_context_t *hal,
			    phy_extra_config_t *phy_extra);
void gmac_phy_802_3_basic_phy_deinit(phy_config_t *phy_cfg, gmac_hal_context_t *hal);

/* MDIO 调试接口（可选） */
int gmac_read_phy_reg(gmac_hal_context_t *hal, uint32_t phy_addr, uint32_t phy_reg,
		      uint32_t *reg_value);
int gmac_write_phy_reg(gmac_hal_context_t *hal, uint32_t phy_addr, uint32_t phy_reg,
		       uint32_t reg_value);

#endif /* GMAC_EQOS_PHY_H */
