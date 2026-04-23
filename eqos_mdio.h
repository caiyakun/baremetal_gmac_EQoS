/*
 * Clause-22 MDIO：通过 EQOS 的 GMAC_MDIO_ADDR / GMAC_MDIO_DATA（0x200/0x204）
 * 访问 PHY。与 Linux stmmac MDIO 位域一致。
 *
 * SPDX-License-Identifier: MIT
 *
 * 注意：
 *   - MDIO_ADDR 的 CR 分频必须使 MDC 落在 PHY 手册允许频率内；eqos_mdio.c
 *     中 mdio_pick_cr_default() 为占位，须按 csr_clk_hz 与 IP 手册改写。
 *   - GB（Go Busy）位超时常见于：MDIO 未接、PHY 未上电、或 CR 过快导致总线异常。
 */
#ifndef EQOS_MDIO_H
#define EQOS_MDIO_H

#include <stdint.h>
#include <stdbool.h>

struct eqos_mdio_cfg {
	/* 与 gmac_hal_context_t.csr_base 一致：MAC CSR 视图，内含 MDIO 寄存器偏移 */
	uintptr_t csr_base;
	/* CSR 时钟 Hz，用于从 csr_clk_hz 推导 MDIO CR 分频（当前 mdio 实现仍为占位 CR） */
	uint32_t csr_clk_hz;
};

int eqos_mdio_c22_read(const struct eqos_mdio_cfg *cfg, uint8_t phy, uint8_t reg,
		       uint16_t *out);
int eqos_mdio_c22_write(const struct eqos_mdio_cfg *cfg, uint8_t phy, uint8_t reg,
			uint16_t val);

#endif /* EQOS_MDIO_H */
