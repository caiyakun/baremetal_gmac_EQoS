/* SPDX-License-Identifier: BSD-2-Clause
 * Synopsys DWC EQoS / DWMAC4 CSR offsets and bit masks (Linux stmmac/dwmac4 对齐).
 * 目标 IP：Ethernet QoS 5.40a 系列（单通道 DMA 常用布局）。
 */
#ifndef EQOS_REGS_H
#define EQOS_REGS_H

#include <stdint.h>

/* --- MAC --- */
#define GMAC_CONFIG			0x0000U
#define GMAC_EXT_CONFIG			0x0004U
#define GMAC_PACKET_FILTER		0x0008U
#define GMAC_RX_FLOW_CTRL		0x0090U
#define GMAC_RXQ_CTRL0		0x00a0U
#define GMAC_RXQ_CTRL1		0x00a4U
#define GMAC_INT_STATUS			0x00b0U
#define GMAC_INT_EN			0x00b4U
#define GMAC_PMT			0x00c0U
#define GMAC_PHYIF_CONTROL_STATUS	0x00f8U
#define GMAC_DEBUG			0x0114U
#define GMAC_HW_FEATURE0		0x011cU
#define GMAC_HW_FEATURE1		0x0120U
#define GMAC_HW_FEATURE2		0x0124U
#define GMAC_HW_FEATURE3		0x0128U
#define GMAC4_VERSION			0x0110U
#define GMAC4_MAC_ONEUS_TIC_COUNTER	0x00dcU
#define GMAC_MDIO_ADDR			0x0200U
#define GMAC_MDIO_DATA			0x0204U
#define GMAC_ADDR_HIGH(reg)		(0x0300U + (unsigned)(reg)*8U)
#define GMAC_ADDR_LOW(reg)		(0x0304U + (unsigned)(reg)*8U)

#define GMAC_CONFIG_TE			(1U << 1)
#define GMAC_CONFIG_RE			(1U << 0)
#define GMAC_CONFIG_DM			(1U << 13)
#define GMAC_CONFIG_LM			(1U << 12)
#define GMAC_CONFIG_PS			(1U << 15)
#define GMAC_CONFIG_FES			(1U << 14)
#define GMAC_CONFIG_JE			(1U << 16)
#define GMAC_CONFIG_JD			(1U << 17)
#define GMAC_CONFIG_BE			(1U << 18)
#define GMAC_CONFIG_DCRS		(1U << 9)
#define GMAC_CONFIG_IPC			(1U << 27)

#define GMAC_HI_REG_AE			(1U << 31)
#define GMAC_HI_DCS_MASK		(7U << 16)
#define GMAC_HI_DCS_SHIFT		16

#define GMAC_PACKET_FILTER_PR		(1U << 0)
#define GMAC_PACKET_FILTER_PM		(1U << 4)
#define GMAC_PACKET_FILTER_RA		(1U << 31)

#define GMAC_PHYIF_CTRLSTATUS_LUD	(1U << 1)

#define GMAC_RX_QUEUE_CLEAR(q)		(~(3U << ((unsigned)(q) * 2U)))
#define GMAC_RX_AV_QUEUE_ENABLE(q)	(1U << ((unsigned)(q) * 2U))
#define GMAC_RX_DCB_QUEUE_ENABLE(q)	(1U << (((unsigned)(q) * 2U) + 1U))

#define MTL_RXQ_DMA_MAP0		0x0c30U
#define MTL_RXQ_DMA_QXMDMACH_MASK(q)	(0xfU << (8U * (unsigned)(q)))
#define MTL_RXQ_DMA_QXMDMACH(ch, q)	(((unsigned)(ch) & 0xfU) << (8U * (unsigned)(q)))

/* --- MTL queue 0 --- */
#define MTL_CHAN_BASE_ADDR		0x0d00U
#define MTL_CHAN_TX_OP_MODE		MTL_CHAN_BASE_ADDR
#define MTL_CHAN_RX_OP_MODE		(MTL_CHAN_BASE_ADDR + 0x30U)

#define MTL_OP_MODE_TSF			(1U << 1)
#define MTL_OP_MODE_RSF			(1U << 5)
#define MTL_OP_MODE_TXQEN		(1U << 3)
#define MTL_OP_MODE_TXQEN_MASK		(3U << 2)
#define MTL_OP_MODE_DIS_TCP_EF		(1U << 6)

/* --- DMA common --- */
#define DMA_BUS_MODE			0x1000U
#define DMA_SYS_BUS_MODE		0x1004U
#define DMA_STATUS			0x1008U
#define DMA_AXI_BUS_MODE		0x1028U

#define DMA_BUS_MODE_SFT_RESET		(1U << 0)
#define DMA_BUS_MODE_DCHE		(1U << 19)

#define DMA_SYS_BUS_FB			(1U << 0)
#define DMA_SYS_BUS_MB			(1U << 14)
#define DMA_SYS_BUS_AAL			(1U << 12)

#define DMA_CHAN_BASE_ADDR		0x1100U
#define DMA_CHAN_BASE_OFFSET		0x80U

#define DMA_CHAN_CONTROL		DMA_CHAN_BASE_ADDR
#define DMA_CHAN_TX_CONTROL		(DMA_CHAN_BASE_ADDR + 0x04U)
#define DMA_CHAN_RX_CONTROL		(DMA_CHAN_BASE_ADDR + 0x08U)
#define DMA_CHAN_TX_BASE_ADDR_HI	(DMA_CHAN_BASE_ADDR + 0x10U)
#define DMA_CHAN_TX_BASE_ADDR		(DMA_CHAN_BASE_ADDR + 0x14U)
#define DMA_CHAN_RX_BASE_ADDR_HI	(DMA_CHAN_BASE_ADDR + 0x18U)
#define DMA_CHAN_RX_BASE_ADDR		(DMA_CHAN_BASE_ADDR + 0x1cU)
#define DMA_CHAN_TX_END_ADDR		(DMA_CHAN_BASE_ADDR + 0x20U)
#define DMA_CHAN_RX_END_ADDR		(DMA_CHAN_BASE_ADDR + 0x28U)
#define DMA_CHAN_TX_RING_LEN		(DMA_CHAN_BASE_ADDR + 0x2cU)
#define DMA_CHAN_RX_RING_LEN		(DMA_CHAN_BASE_ADDR + 0x30U)
#define DMA_CHAN_INTR_ENA		(DMA_CHAN_BASE_ADDR + 0x34U)
#define DMA_CHAN_STATUS			(DMA_CHAN_BASE_ADDR + 0x60U)

#define DMA_CHAN_CTRL_PBLX8		(1U << 16)
#define DMA_CONTROL_ST			(1U << 0)
#define DMA_CONTROL_SR			(1U << 0)
#define DMA_CONTROL_OSP			(1U << 4)
#define DMA_CONTROL_TSE			(1U << 12)

#define DMA_CHAN_TX_CTRL_TXPBL_MASK	(0x3fU << 16)
#define DMA_CHAN_RX_CTRL_RXPBL_MASK	(0x3fU << 16)
#define DMA_RBSZ_MASK			(0x7fffU << 1)

/* 4.10a+ 中断使能布局 (stmmac: DMA_CHAN_INTR_DEFAULT_MASK_4_10) */
#define DMA_CHAN_INTR_ENA_NIE_4_10	(1U << 15)
#define DMA_CHAN_INTR_ENA_AIE_4_10	(1U << 14)
#define DMA_CHAN_INTR_ENA_FBE		(1U << 12)
#define DMA_CHAN_INTR_ENA_RPS		(1U << 8)
#define DMA_CHAN_INTR_ENA_RBU		(1U << 7)
#define DMA_CHAN_INTR_ENA_RIE		(1U << 6)
#define DMA_CHAN_INTR_ENA_TIE		(1U << 0)

#define DMA_CHAN_INTR_DEFAULT_MASK_4_10 \
	(DMA_CHAN_INTR_ENA_NIE_4_10 | DMA_CHAN_INTR_ENA_AIE_4_10 | \
	 DMA_CHAN_INTR_ENA_FBE | DMA_CHAN_INTR_ENA_RPS | \
	 DMA_CHAN_INTR_ENA_RBU | DMA_CHAN_INTR_ENA_RIE | \
	 DMA_CHAN_INTR_ENA_TIE)

#define DMA_CHAN_STATUS_NIS		(1U << 15)
#define DMA_CHAN_STATUS_AIS		(1U << 14)
#define DMA_CHAN_STATUS_RI		(1U << 6)
#define DMA_CHAN_STATUS_TI		(1U << 0)

#define DMA_HW_FEAT_ACTPHYIF_MASK	(7U << 28)

#endif /* EQOS_REGS_H */
