/* SPDX-License-Identifier: MIT */
/*
 * Clause-22 读/写流程（与 Linux stmmac 一致）：
 *   1) 等 GB=0（上一笔事务结束）。
 *   2) 写 MDIO_DATA（写周期）；读周期常写 0。
 *   3) 组 MDIO_ADDR：PA、RDA、GOC（读/写编码）、CR（MDC 分频）、最后置 GB。
 *   4) 再等 GB=0，从 MDIO_DATA 取 16bit 数据（读周期）。
 *
 * 为何写两次 *ma = w; *ma = w | MDIO_GB：
 *   部分集成要求先写除 GB 外的字段再单独或门 GB 触发，与内核写法对齐。
 *
 * 关闭 MDIO 调试打印：与 gmac_eqos_compat.c 相同，编译加 -DGMAC_EQOS_SILENT
 * 或自定义 #define MDIO_BRUP(...) your_uart_printf(__VA_ARGS__)
 */
#include "eqos_mdio.h"
#include "eqos_dwmac4_hw.h"

#if defined(GMAC_EQOS_SILENT)
#define MDIO_BRUP(...) ((void)0)
#elif !defined(MDIO_BRUP)
#include <stdio.h>
#define MDIO_BRUP(...) printf(__VA_ARGS__)
#endif

#define MDIO_GB			BIT(0)
#define MDIO_C45E		BIT(1)
#define MDIO_GOC_WR		BIT(2)
#define MDIO_GOC_RD		(3u << 2)
#define MDIO_PA_SHIFT		21
#define MDIO_RDA_SHIFT		16
#define MDIO_CR_SHIFT		8

/* 指针形式 REG32：与 gmac_eqos_compat 中 *(volatile uint32_t*) 等价，便于取 & 轮询 GB */
#define REG32(b, o) ((volatile uint32_t *)((uintptr_t)(b) + (uintptr_t)(o)))

static int wait_gb_clear(volatile uint32_t *ma, unsigned spins)
{
	while ((*ma & MDIO_GB) != 0u) {
		if (spins-- == 0u)
			return -1;
	}
	return 0;
}

/*
 * MDC 分频 CR：值与 IP 版本/集成有关，须满足
 *   MDC_freq = csr_clk_hz / (2 * divider_table[CR])
 * 且不超过 PHY（如 VCS8541）手册的 MDC 上限。占位值错误会导致 GB 永远忙或
 * 读回全 0/全 1。
 */
static uint32_t mdio_pick_cr_default(void)
{
	return (3u << MDIO_CR_SHIFT);
}

int eqos_mdio_c22_read(const struct eqos_mdio_cfg *cfg, uint8_t phy, uint8_t reg,
		       uint16_t *out)
{
	volatile uint32_t *ma = REG32(cfg->csr_base, GMAC_MDIO_ADDR);
	volatile uint32_t *md = REG32(cfg->csr_base, GMAC_MDIO_DATA);

	(void)cfg->csr_clk_hz;
	if (wait_gb_clear(ma, 1000000u) != 0) {
		MDIO_BRUP("[EQOS-MDIO] read: GB busy before command (phy=%u reg=%u)\n",
			  phy, reg);
		return -1;
	}

	*md = 0u;
	{
		uint32_t w = mdio_pick_cr_default();

		w |= ((uint32_t)phy & 0x1Fu) << MDIO_PA_SHIFT;
		w |= ((uint32_t)reg & 0x1Fu) << MDIO_RDA_SHIFT;
		w |= MDIO_GOC_RD;
		*ma = w;
		*ma = w | MDIO_GB;
	}

	if (wait_gb_clear(ma, 1000000u) != 0) {
		MDIO_BRUP("[EQOS-MDIO] read: GB busy after read (phy=%u reg=%u)\n",
			  phy, reg);
		return -1;
	}

	*out = (uint16_t)(*md & 0xFFFFu);
	return 0;
}

int eqos_mdio_c22_write(const struct eqos_mdio_cfg *cfg, uint8_t phy, uint8_t reg,
			uint16_t val)
{
	volatile uint32_t *ma = REG32(cfg->csr_base, GMAC_MDIO_ADDR);
	volatile uint32_t *md = REG32(cfg->csr_base, GMAC_MDIO_DATA);

	if (wait_gb_clear(ma, 1000000u) != 0) {
		MDIO_BRUP("[EQOS-MDIO] write: GB busy before command (phy=%u reg=%u)\n",
			  phy, reg);
		return -1;
	}

	*md = (uint32_t)val & 0xFFFFu;
	{
		uint32_t w = mdio_pick_cr_default();

		w |= ((uint32_t)phy & 0x1Fu) << MDIO_PA_SHIFT;
		w |= ((uint32_t)reg & 0x1Fu) << MDIO_RDA_SHIFT;
		w |= MDIO_GOC_WR;
		*ma = w;
		*ma = w | MDIO_GB;
	}

	if (wait_gb_clear(ma, 1000000u) != 0) {
		MDIO_BRUP("[EQOS-MDIO] write: GB busy after write (phy=%u reg=%u val=0x%04x)\n",
			  phy, reg, val);
		return -1;
	}

	return 0;
}
