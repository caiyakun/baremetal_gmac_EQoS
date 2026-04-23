/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Synopsys DWMAC v4.x / EQOS 常用 CSR 与描述符位定义。
 *
 * 为何与 Linux 对齐：
 *   偏移与位掩码直接对照 drivers/net/ethernet/stmicro/stmmac 下
 *   dwmac4.h / dwmac4_dma.h / dwmac4_descs.h，便于用内核驱动与设备树
 *   反查你板级集成差异（复位、时钟、strap、队列数）。
 *
 * 使用注意：
 *   - 部分 SoC 在 CSR 前加「外设 wrapper」额外寄存器，此时 csr_base 应指向
 *     内核 dt 中 snps,dwmac-4.xx 节点的 reg，而非 wrapper 根。
 *   - 描述符 OWN 位语义：1=硬件占有，0=软件可写；Tx 与 Rx 方向相反于「谁
 *     提交」的直觉，填环时务必对照手册与 stmmac 代码路径。
 */
#ifndef EQOS_DWMAC4_HW_H
#define EQOS_DWMAC4_HW_H

#include <stdint.h>

#ifndef BIT
#define BIT(n) (1u << (n))
#endif
#ifndef GENMASK
#define GENMASK(h, l) (((1u << ((h) - (l) + 1)) - 1u) << (l))
#endif

/* --- MAC ---------------------------------------------------------------- */
#define GMAC_CONFIG			0x00000000u
#define GMAC_EXT_CONFIG			0x00000004u
#define GMAC_PACKET_FILTER		0x00000008u
#define GMAC_MDIO_ADDR			0x00000200u
#define GMAC_MDIO_DATA			0x00000204u
#define GMAC_ADDR_HIGH(reg)		(0x00000300u + (unsigned)(reg)*8u)
#define GMAC_ADDR_LOW(reg)		(0x00000304u + (unsigned)(reg)*8u)
#define GMAC_VERSION			0x00000020u /* 旧版 GMAC */
#define GMAC_HW_FEATURE1		0x00000120u
#define GMAC_HW_FEATURE2		0x00000124u
#define GMAC4_VERSION			0x00000110u /* GMAC4+ / DWMAC4，见 Linux hwif.h */

#define GMAC_PACKET_FILTER_RA		BIT(31)
#define GMAC_PACKET_FILTER_PM		BIT(4)

#define GMAC_HI_REG_AE			BIT(31)

/* TE/RE：MAC 侧发/收使能，常与 DMA ST/SR 成对打开。
 * DM：双工模式；PS：端口选择（MII 8bit 等）；FES：FE 速度选择，与千兆配置配合。
 * LM：MAC 环回，bring-up 常用，外接 PHY 时必须关闭。 */
#define GMAC_CONFIG_TE			BIT(1)
#define GMAC_CONFIG_RE			BIT(0)
#define GMAC_CONFIG_DM			BIT(13)
#define GMAC_CONFIG_PS			BIT(15)
#define GMAC_CONFIG_FES			BIT(14)
#define GMAC_CONFIG_LM			BIT(12)

#define GMAC_HW_TXFIFOSIZE		GENMASK(10, 6)
#define GMAC_HW_RXFIFOSIZE		GENMASK(4, 0)

/* --- MTL ---------------------------------------------------------------- */
#define MTL_OPERATION_MODE		0x00000c00u
#define MTL_RXQ_DMA_MAP0		0x00000c30u
#define MTL_RXQ_DMA_MAP1		0x00000c34u

#define MTL_OPERATION_RAA		BIT(2)
#define MTL_OPERATION_RAA_SP		(0u << 2)
#define MTL_OPERATION_RAA_WSP		BIT(2)
#define MTL_OPERATION_SCHALG_MASK	GENMASK(6, 5)
#define MTL_OPERATION_SCHALG_SP		(3u << 5)

#define MTL_RXQ_DMA_QXMDMACH_MASK(q)	(0xfu << (8u * (unsigned)(q)))
#define MTL_RXQ_DMA_QXMDMACH(ch, q)	(((unsigned)(ch)&0xfu) << (8u * (unsigned)(q)))

#define MTL_CHAN_BASE_ADDR		0x00000d00u
#define MTL_CHAN_BASE_OFFSET		0x40u

#define MTL_OP_MODE_TXQEN_MASK		GENMASK(3, 2)
#define MTL_OP_MODE_TXQEN		BIT(3)
#define MTL_OP_MODE_TXQEN_AV		BIT(2)
#define MTL_OP_MODE_TSF			BIT(1)
#define MTL_OP_MODE_TQS_MASK		GENMASK(24, 16)
#define MTL_OP_MODE_TQS_SHIFT		16
#define MTL_OP_MODE_RSF			BIT(5)
#define MTL_OP_MODE_RQS_MASK		GENMASK(29, 20)
#define MTL_OP_MODE_RQS_SHIFT		20

/* --- DMA (csr 内偏移，与 Linux DMA_BUS_MODE 基址一致) -------------------- */
#define DMA_BUS_MODE			0x00001000u
#define DMA_SYS_BUS_MODE		0x00001004u
#define DMA_STATUS			0x00001008u

#define DMA_BUS_MODE_SFT_RESET		BIT(0)

/* 允许的最大 AXI burst 长度集合；若从机最大突发小于 256，应删高位选项
 * 以免 AXI DECERR。FB：fixed burst 相关能力位，与集成有关。 */
#define DMA_SYS_BUS_FB			BIT(0)
#define DMA_AXI_BLEN256			BIT(7)
#define DMA_AXI_BLEN128			BIT(6)
#define DMA_AXI_BLEN64			BIT(5)
#define DMA_AXI_BLEN32			BIT(4)
#define DMA_AXI_BLEN16			BIT(3)
#define DMA_AXI_BLEN8			BIT(2)
#define DMA_AXI_BLEN4			BIT(1)
#define DMA_BURST_LEN_DEFAULT		(DMA_AXI_BLEN256 | DMA_AXI_BLEN128 | \
					DMA_AXI_BLEN64 | DMA_AXI_BLEN32 | \
					DMA_AXI_BLEN16 | DMA_AXI_BLEN8 | \
					DMA_AXI_BLEN4)

#define DMA_CHAN_BASE_ADDR		0x00001100u
#define DMA_CHAN_BASE_OFFSET		0x80u

#define DMA_CHAN_CONTROL(ch)		(DMA_CHAN_BASE_ADDR + (ch)*DMA_CHAN_BASE_OFFSET + 0x00u)
#define DMA_CHAN_TX_CONTROL(ch)		(DMA_CHAN_BASE_ADDR + (ch)*DMA_CHAN_BASE_OFFSET + 0x04u)
#define DMA_CHAN_RX_CONTROL(ch)		(DMA_CHAN_BASE_ADDR + (ch)*DMA_CHAN_BASE_OFFSET + 0x08u)
#define DMA_CHAN_TX_BASE_ADDR_HI(ch)	(DMA_CHAN_BASE_ADDR + (ch)*DMA_CHAN_BASE_OFFSET + 0x10u)
#define DMA_CHAN_TX_BASE_ADDR(ch)	(DMA_CHAN_BASE_ADDR + (ch)*DMA_CHAN_BASE_OFFSET + 0x14u)
#define DMA_CHAN_RX_BASE_ADDR_HI(ch)	(DMA_CHAN_BASE_ADDR + (ch)*DMA_CHAN_BASE_OFFSET + 0x18u)
#define DMA_CHAN_RX_BASE_ADDR(ch)	(DMA_CHAN_BASE_ADDR + (ch)*DMA_CHAN_BASE_OFFSET + 0x1cu)
#define DMA_CHAN_TX_END_ADDR(ch)	(DMA_CHAN_BASE_ADDR + (ch)*DMA_CHAN_BASE_OFFSET + 0x20u)
#define DMA_CHAN_RX_END_ADDR(ch)	(DMA_CHAN_BASE_ADDR + (ch)*DMA_CHAN_BASE_OFFSET + 0x28u)
#define DMA_CHAN_TX_RING_LEN(ch)	(DMA_CHAN_BASE_ADDR + (ch)*DMA_CHAN_BASE_OFFSET + 0x2cu)
#define DMA_CHAN_RX_RING_LEN(ch)	(DMA_CHAN_BASE_ADDR + (ch)*DMA_CHAN_BASE_OFFSET + 0x30u)
#define DMA_CHAN_INTR_ENA(ch)		(DMA_CHAN_BASE_ADDR + (ch)*DMA_CHAN_BASE_OFFSET + 0x34u)

/* ST/SR 在不同寄存器（TX_CONTROL / RX_CONTROL）里同名 BIT(0)，含义分别为
 * Tx DMA Start、Rx DMA Start。 */
#define DMA_CONTROL_ST			BIT(0)
#define DMA_CONTROL_SR			BIT(0)
#define DMA_CONTROL_OSP			BIT(4)
#define DMA_BUS_MODE_PBL		BIT(16)
#define DMA_BUS_MODE_PBL_SHIFT		16
#define DMA_BUS_MODE_RPBL_SHIFT		16
#define DMA_RBSZ_MASK			GENMASK(14, 1)
#define DMA_RBSZ_SHIFT			1u

/*
 * DWMAC4 描述符：读侧（软件填）与写回（硬件更新 OWN/长度/错误位）共用
 * 同一四 dword 布局。具体 des2/des3 各域随增强描述符（read / write-back
 * split）模式变化；本裸机代码使用最简「read = write-back」形态。
 */
struct eqos_dma_desc {
	uint32_t des0;
	uint32_t des1;
	uint32_t des2;
	uint32_t des3;
};

#define TDES2_BUFFER1_SIZE_MASK		GENMASK(13, 0)
#define TDES2_INTERRUPT_ON_COMPLETION	BIT(31)

#define TDES3_PACKET_SIZE_MASK		GENMASK(14, 0)
#define TDES3_CHECKSUM_INSERTION_MASK	GENMASK(17, 16)
#define TDES3_CHECKSUM_INSERTION_SHIFT	16u
#define TDES3_LAST_DESCRIPTOR		BIT(28)
#define TDES3_FIRST_DESCRIPTOR		BIT(29)
#define TDES3_OWN			BIT(31)

#define RDES3_PACKET_SIZE_MASK		GENMASK(14, 0)
#define RDES3_ERROR_SUMMARY		BIT(15)
#define RDES3_LAST_DESCRIPTOR		BIT(28)
#define RDES3_FIRST_DESCRIPTOR		BIT(29)
/* Rx 数据描述符与 PTP/Offload 产生的「上下文」描述符共用 des3 格式；
 * CONTEXT=1 时 des0..2 含时间戳等，非以太帧，应用层须跳过。 */
#define RDES3_CONTEXT_DESCRIPTOR	BIT(30)
#define RDES3_BUFFER1_VALID_ADDR	BIT(24)
/*
 * 以下位在部分手册中与 CONTEXT 同处 bit30 的不同描述符类型中互斥出现；
 * 本驱动 Rx 路径仅用 CONTEXT + FIRST/LAST，勿把二者同时当真。
 */
#define RDES3_INT_ON_COMPLETION_EN	BIT(30)
#define RDES3_OWN			BIT(31)

#endif /* EQOS_DWMAC4_HW_H */
