/* SPDX-License-Identifier: BSD-2-Clause
 * Synopsys DWC EQoS / DWMAC4 CSR offsets and bit masks (Linux stmmac/dwmac4 对齐).
 * 目标 IP：Ethernet QoS 5.40a 系列（单通道 DMA 常用布局）。
 */
#ifndef EQOS_REGS_H
#define EQOS_REGS_H

#include <stdint.h>

/*
 * --- MAC CSR 区域 ---
 *
 * 这些 offset 都是相对 EQoS CSR 基址（gmac_hal_context_t.csr_base）的偏移。
 * 命名基本跟 Synopsys databook / Linux stmmac(dwmacc4) 保持一致。
 */

/* MAC Configuration：MAC 主控制寄存器，控制 TE/RE、速率、双工、环回、Jumbo、checksum 等。 */
#define GMAC_CONFIG			0x0000U
/* MAC Extended Configuration：扩展 MAC 配置，本裸机路径当前未使用。 */
#define GMAC_EXT_CONFIG			0x0004U
/* MAC Packet Filter：包过滤配置，控制混杂模式、广播/组播、Receive-All 等过滤行为。 */
#define GMAC_PACKET_FILTER		0x0008U
/* MAC RX Flow Control：RX 流控/PAUSE 相关配置，当前 near-end loopback 未使用。 */
#define GMAC_RX_FLOW_CTRL		0x0090U
/* MAC RX Queue Control 0：每个 RX queue 的使能模式，queue0 常用 DCB/AV 模式。 */
#define GMAC_RXQ_CTRL0			0x00a0U
/* MAC RX Queue Control 1：RX queue 扩展控制，当前单 queue 未使用。 */
#define GMAC_RXQ_CTRL1			0x00a4U
/* MAC Interrupt Status：MAC 侧中断状态，区别于 DMA channel 中断状态。 */
#define GMAC_INT_STATUS			0x00b0U
/* MAC Interrupt Enable：MAC 侧中断使能寄存器。 */
#define GMAC_INT_EN			0x00b4U
/* Power Management：Magic Packet / Wake-on-LAN / PMT 相关控制。 */
#define GMAC_PMT			0x00c0U
/* PHY Interface Control and Status：PHY/MAC 接口状态，RGMII link update 等在这里。 */
#define GMAC_PHYIF_CONTROL_STATUS	0x00f8U
/* MAC Version：DWMAC4/EQoS 版本号，读出来可确认 IP 版本。 */
#define GMAC4_VERSION			0x0110U
/* MAC Debug：MAC/MTL 内部状态调试寄存器，用于定位 TX/RX 状态机卡住位置。 */
#define GMAC_DEBUG			0x0114U
/* Hardware Feature 0~3：硬件能力寄存器，描述队列数、PTP、TSO、PHY interface 等综合选项。 */
#define GMAC_HW_FEATURE0		0x011cU
#define GMAC_HW_FEATURE1		0x0120U
#define GMAC_HW_FEATURE2		0x0124U
#define GMAC_HW_FEATURE3		0x0128U
/* MAC 1us Tick Counter：写入 csr_clk/1MHz - 1，使 MAC 内部 1us tick 与实际 CSR 时钟对齐。 */
#define GMAC4_MAC_ONEUS_TIC_COUNTER	0x00dcU
/* MDIO Address：GMAC4 MDIO 控制/地址寄存器；bit0 busy，bit[3:2] GOC，bit[25:21] PHY addr。 */
#define GMAC_MDIO_ADDR			0x0200U
/* MDIO Data：MDIO Clause 22 读写数据寄存器，低 16 位有效。 */
#define GMAC_MDIO_DATA			0x0204U
/* MAC Address High(n)：MAC 地址槽 n 的高 16 位 + Address Enable + DMA channel select。 */
#define GMAC_ADDR_HIGH(reg)		(0x0300U + (unsigned)(reg) * 8U)
/* MAC Address Low(n)：MAC 地址槽 n 的低 32 位。 */
#define GMAC_ADDR_LOW(reg)		(0x0304U + (unsigned)(reg) * 8U)

/* GMAC_CONFIG.RE：Receiver Enable，置位后 MAC 接收状态机开始接收帧。 */
#define GMAC_CONFIG_RE			(1U << 0)
/* GMAC_CONFIG.TE：Transmitter Enable，置位后 MAC 发送状态机允许发送帧。 */
#define GMAC_CONFIG_TE			(1U << 1)
/* GMAC_CONFIG.DCRS：Disable Carrier Sense，半双工时禁用 CRS 检查，调试环回时减少外部依赖。 */
#define GMAC_CONFIG_DCRS		(1U << 9)
/* GMAC_CONFIG.LM：Loopback Mode，MAC 内部近端环回，TX 帧直接回到 RX 路径。 */
#define GMAC_CONFIG_LM			(1U << 12)
/* GMAC_CONFIG.DM：Duplex Mode，1=全双工，0=半双工。 */
#define GMAC_CONFIG_DM			(1U << 13)
/* GMAC_CONFIG.FES：Fast Ethernet Speed；配合 PS 使用，PS=1/FES=1 表示 100M。 */
#define GMAC_CONFIG_FES			(1U << 14)
/* GMAC_CONFIG.PS：Port Select；1=10/100M，0=1000M。10M/100M 再由 FES 区分。 */
#define GMAC_CONFIG_PS			(1U << 15)
/* GMAC_CONFIG.JE：Jumbo Packet Enable，允许 Jumbo 帧相关路径。 */
#define GMAC_CONFIG_JE			(1U << 16)
/* GMAC_CONFIG.JD：Jabber Disable，关闭超长发送 jabber 检测限制。 */
#define GMAC_CONFIG_JD			(1U << 17)
/* GMAC_CONFIG.BE：Frame Burst Enable，使能帧 burst 发送能力。 */
#define GMAC_CONFIG_BE			(1U << 18)
/* GMAC_CONFIG.IPC：Checksum Offload，打开 IPv4/TCP/UDP/ICMP checksum 检查/卸载路径。 */
#define GMAC_CONFIG_IPC			(1U << 27)

/* GMAC_ADDR_HIGH.AE：Address Enable，置位后该 MAC 地址槽参与地址过滤匹配。 */
#define GMAC_HI_REG_AE			(1U << 31)
/* GMAC_ADDR_HIGH.DCS：DMA Channel Select，地址命中后路由到哪个 DMA channel。 */
#define GMAC_HI_DCS_MASK		(7U << 16)
/* GMAC_ADDR_HIGH.DCS 字段移位。 */
#define GMAC_HI_DCS_SHIFT		16

/* GMAC_PACKET_FILTER.PR：Promiscuous Mode，接收所有帧，不受 DA 过滤限制。 */
#define GMAC_PACKET_FILTER_PR		(1U << 0)
/* GMAC_PACKET_FILTER.PM：Pass All Multicast，接收所有组播帧。 */
#define GMAC_PACKET_FILTER_PM		(1U << 4)
/* GMAC_PACKET_FILTER.RA：Receive All，把过滤失败的帧也传给应用/DMA。 */
#define GMAC_PACKET_FILTER_RA		(1U << 31)

/* GMAC_PHYIF_CONTROL_STATUS.LUD：Link Up/Down，通知 MAC PHY 链路状态有更新。 */
#define GMAC_PHYIF_CTRLSTATUS_LUD	(1U << 1)

/*
 * GMAC_RXQ_CTRL0 每个 RX queue 占 2 bit：
 *   00 = disabled
 *   01 = AV queue enable
 *   10 = DCB queue enable
 */
/* 清除 RX queue q 的 2-bit enable 模式字段。 */
#define GMAC_RX_QUEUE_CLEAR(q)		(~(3U << ((unsigned)(q) * 2U)))
/* 把 RX queue q 设置为 AV(Audio Video) queue 模式。 */
#define GMAC_RX_AV_QUEUE_ENABLE(q)	(1U << ((unsigned)(q) * 2U))
/* 把 RX queue q 设置为 DCB(Data Center Bridging) queue 模式，单队列常用此模式。 */
#define GMAC_RX_DCB_QUEUE_ENABLE(q)	(1U << (((unsigned)(q) * 2U) + 1U))

/*
 * --- MTL 全局 / Queue 映射 ---
 *
 * MTL = MAC Transaction Layer，位于 MAC 与 DMA 之间，包含 TX/RX FIFO、queue 调度、
 * RX queue 到 DMA channel 的映射等。
 */

/* MTL RX Queue DMA Map 0：配置 RX queue0~3 分别映射到哪个 DMA channel。 */
#define MTL_RXQ_DMA_MAP0		0x0c30U
/* RX queue q 的 DMA channel 映射字段 mask，每个 queue 占 4 bit。 */
#define MTL_RXQ_DMA_QXMDMACH_MASK(q)	(0xfU << (8U * (unsigned)(q)))
/* 设置 RX queue q 映射到 DMA channel ch。 */
#define MTL_RXQ_DMA_QXMDMACH(ch, q)	(((unsigned)(ch) & 0xfU) << (8U * (unsigned)(q)))

/* --- MTL queue 0 寄存器 --- */

/* MTL queue0 基址；多 queue 时通常每个 queue 有固定 stride。 */
#define MTL_CHAN_BASE_ADDR		0x0d00U
/* MTL TX Queue Operation Mode：TX FIFO/Store-Forward/queue enable 等控制。 */
#define MTL_CHAN_TX_OP_MODE		MTL_CHAN_BASE_ADDR
/* MTL RX Queue Operation Mode：RX FIFO/Store-Forward/阈值等控制。 */
#define MTL_CHAN_RX_OP_MODE		(MTL_CHAN_BASE_ADDR + 0x30U)

/* MTL_TXQ_OP_MODE.TSF：Transmit Store and Forward，整帧进 FIFO 后再发。 */
#define MTL_OP_MODE_TSF			(1U << 1)
/* MTL_RXQ_OP_MODE.RSF：Receive Store and Forward，完整帧进 RX FIFO 后再交给 DMA。 */
#define MTL_OP_MODE_RSF			(1U << 5)
/* MTL_TXQ_OP_MODE.TXQEN：TX queue enable 的 bit3，配合 TXQEN_MASK 写 queue 使能模式。 */
#define MTL_OP_MODE_TXQEN		(1U << 3)
/* MTL_TXQ_OP_MODE.TXQEN 字段 mask，2 bit 宽。 */
#define MTL_OP_MODE_TXQEN_MASK		(3U << 2)
/* MTL_TXQ_OP_MODE.TTC 字段 mask：TX FIFO threshold control，禁用 TSF 后决定 FIFO 到多少字节开始发。 */
#define MTL_OP_MODE_TTC_MASK		(7U << 4)
/* MTL_TXQ_OP_MODE.TTC=1：TX threshold 64 bytes；对齐 P5 gmac_hal_init_dma_default 的 64B 阈值。 */
#define MTL_OP_MODE_TTC_64		(1U << 4)
/* MTL_RXQ_OP_MODE.RTC 字段 mask：RX FIFO threshold control，禁用 RSF 后决定 FIFO 到多少字节交 DMA。 */
#define MTL_OP_MODE_RTC_MASK		(3U << 0)
/* MTL_RXQ_OP_MODE.RTC=0：RX threshold 64 bytes；对齐 P5 gmac_hal_init_dma_default 的 64B 阈值。 */
#define MTL_OP_MODE_RTC_64		(0U << 0)
/* MTL_RXQ_OP_MODE.DIS_TCP_EF：禁用 TCP/IP checksum error frame 转发，丢弃 checksum 错误帧。 */
#define MTL_OP_MODE_DIS_TCP_EF		(1U << 6)

/*
 * --- DMA common 寄存器 ---
 *
 * DWMAC4/EQoS DMA 分 common 与 channel 两层。common 负责全局 bus/reset，
 * channel 负责具体 TX/RX descriptor ring 与启动停止。
 */

/* DMA Bus Mode：DMA common 主控制，含软件复位、descriptor cache 等。 */
#define DMA_BUS_MODE			0x1000U
/* DMA System Bus Mode：系统总线 AXI/AHB burst、地址对齐等全局 bus 行为。 */
#define DMA_SYS_BUS_MODE		0x1004U
/* DMA Status：DMA common 状态寄存器，当前单 channel 路径主要用 channel status。 */
#define DMA_STATUS			0x1008U
/* DMA AXI Bus Mode：AXI 专用 bus 配置，部分集成使用该寄存器扩展 outstanding/burst。 */
#define DMA_AXI_BUS_MODE		0x1028U

/* DMA_BUS_MODE.SFT_RESET：DMA 软件复位；置 1 后硬件复位 DMA 并自动清 0。 */
#define DMA_BUS_MODE_SFT_RESET		(1U << 0)
/* DMA_BUS_MODE.DCHE：Descriptor Cache Enable；本裸机手动维护 cache，因此清零。 */
#define DMA_BUS_MODE_DCHE		(1U << 19)

/* DMA_SYS_BUS_MODE.FB：Fixed Burst，使用固定 burst 长度访问系统总线。 */
#define DMA_SYS_BUS_FB			(1U << 0)
/* DMA_SYS_BUS_MODE.MB：Mixed Burst，允许 mixed burst；当前代码未置位。 */
#define DMA_SYS_BUS_MB			(1U << 14)
/* DMA_SYS_BUS_MODE.AAL：Address-Aligned Beats，burst 按地址边界对齐。 */
#define DMA_SYS_BUS_AAL			(1U << 12)

/*
 * --- DMA channel 0 寄存器 ---
 *
 * 当前裸机 HAL 只使用 channel0。DWMAC4 多 channel 时每个 channel 通常相隔
 * DMA_CHAN_BASE_OFFSET。
 */

/* DMA channel0 基址。 */
#define DMA_CHAN_BASE_ADDR		0x1100U
/* DMA channel stride：多 channel 时 channel n = base + n * offset。 */
#define DMA_CHAN_BASE_OFFSET		0x80U

/* DMA Channel Control：channel 通用控制，如 PBLx8 等。 */
#define DMA_CHAN_CONTROL		DMA_CHAN_BASE_ADDR
/* DMA Channel TX Control：TX DMA 启停、TXPBL、TSE、OSP 等。 */
#define DMA_CHAN_TX_CONTROL		(DMA_CHAN_BASE_ADDR + 0x04U)
/* DMA Channel RX Control：RX DMA 启停、RXPBL、RBSZ 等。 */
#define DMA_CHAN_RX_CONTROL		(DMA_CHAN_BASE_ADDR + 0x08U)
/* TX descriptor list base high 32 bits。 */
#define DMA_CHAN_TX_BASE_ADDR_HI	(DMA_CHAN_BASE_ADDR + 0x10U)
/* TX descriptor list base low 32 bits。 */
#define DMA_CHAN_TX_BASE_ADDR		(DMA_CHAN_BASE_ADDR + 0x14U)
/* RX descriptor list base high 32 bits。 */
#define DMA_CHAN_RX_BASE_ADDR_HI	(DMA_CHAN_BASE_ADDR + 0x18U)
/* RX descriptor list base low 32 bits。 */
#define DMA_CHAN_RX_BASE_ADDR		(DMA_CHAN_BASE_ADDR + 0x1cU)
/* TX descriptor tail pointer；写入某 TX 描述符地址后通知 DMA 可取新 TX 描述符。 */
#define DMA_CHAN_TX_END_ADDR		(DMA_CHAN_BASE_ADDR + 0x20U)
/* RX descriptor tail pointer；写入某 RX 描述符地址后通知 DMA RX ring 可用窗口。 */
#define DMA_CHAN_RX_END_ADDR		(DMA_CHAN_BASE_ADDR + 0x28U)
/* TX ring length，DWMAC4 写 “描述符数量 - 1”。 */
#define DMA_CHAN_TX_RING_LEN		(DMA_CHAN_BASE_ADDR + 0x2cU)
/* RX ring length，DWMAC4 写 “描述符数量 - 1”。 */
#define DMA_CHAN_RX_RING_LEN		(DMA_CHAN_BASE_ADDR + 0x30U)
/* DMA channel interrupt enable：TX/RX complete、异常、fatal bus error 等使能。 */
#define DMA_CHAN_INTR_ENA		(DMA_CHAN_BASE_ADDR + 0x34U)
/* DMA channel status：TX/RX complete、normal/abnormal summary 等状态，通常 W1C 清除。 */
#define DMA_CHAN_STATUS			(DMA_CHAN_BASE_ADDR + 0x60U)

/* DMA_CHAN_CONTROL.PBLX8：置位后 PBL 字段乘 8；清零表示 PBL 字段直接作为 burst length。 */
#define DMA_CHAN_CTRL_PBLX8		(1U << 16)
/* DMA_CHAN_TX_CONTROL.ST：Start/Stop Transmission Command，置位启动 TX DMA。 */
#define DMA_CONTROL_ST			(1U << 0)
/* DMA_CHAN_RX_CONTROL.SR：Start/Stop Receive Command，置位启动 RX DMA。 */
#define DMA_CONTROL_SR			(1U << 0)
/* DMA_CHAN_TX_CONTROL.OSP：Operate on Second Packet，提高连续发包吞吐。 */
#define DMA_CONTROL_OSP			(1U << 4)
/* DMA_CHAN_TX_CONTROL.TSE：TCP Segmentation Enable；普通以太帧测试应清零。 */
#define DMA_CONTROL_TSE			(1U << 12)

/* DMA_CHAN_TX_CONTROL.TXPBL 字段 mask：TX Programmable Burst Length，bit[21:16]。 */
#define DMA_CHAN_TX_CTRL_TXPBL_MASK	(0x3fU << 16)
/* DMA_CHAN_RX_CONTROL.RXPBL 字段 mask：RX Programmable Burst Length，bit[21:16]。 */
#define DMA_CHAN_RX_CTRL_RXPBL_MASK	(0x3fU << 16)
/* DMA_CHAN_RX_CONTROL.RBSZ 字段 mask：Receive Buffer Size 编码字段，bit[15:1]。 */
#define DMA_RBSZ_MASK			(0x7fffU << 1)

/* 4.10a+ channel interrupt enable 位布局 (stmmac: DMA_CHAN_INTR_DEFAULT_MASK_4_10)。 */
/* DMA_CHAN_INTR_ENA.NIE：Normal Interrupt Summary Enable。 */
#define DMA_CHAN_INTR_ENA_NIE_4_10	(1U << 15)
/* DMA_CHAN_INTR_ENA.AIE：Abnormal Interrupt Summary Enable。 */
#define DMA_CHAN_INTR_ENA_AIE_4_10	(1U << 14)
/* DMA_CHAN_INTR_ENA.FBE：Fatal Bus Error Enable。 */
#define DMA_CHAN_INTR_ENA_FBE		(1U << 12)
/* DMA_CHAN_INTR_ENA.RPS：Receive Process Stopped Enable。 */
#define DMA_CHAN_INTR_ENA_RPS		(1U << 8)
/* DMA_CHAN_INTR_ENA.RBU：Receive Buffer Unavailable Enable。 */
#define DMA_CHAN_INTR_ENA_RBU		(1U << 7)
/* DMA_CHAN_INTR_ENA.RIE：Receive Interrupt Enable，RX 完成中断。 */
#define DMA_CHAN_INTR_ENA_RIE		(1U << 6)
/* DMA_CHAN_INTR_ENA.TIE：Transmit Interrupt Enable，TX 完成中断。 */
#define DMA_CHAN_INTR_ENA_TIE		(1U << 0)

/*
 * 默认打开的 DMA channel 中断集合：
 * - 正常/异常 summary
 * - fatal bus error
 * - RX process stopped / RX buffer unavailable
 * - RX complete / TX complete
 * 当前测试以轮询为主，但打开这些位后可通过 DMA_CHAN_STATUS 观察和清除 RI/TI。
 */
#define DMA_CHAN_INTR_DEFAULT_MASK_4_10 \
	(DMA_CHAN_INTR_ENA_NIE_4_10 | DMA_CHAN_INTR_ENA_AIE_4_10 | \
	 DMA_CHAN_INTR_ENA_FBE | DMA_CHAN_INTR_ENA_RPS | \
	 DMA_CHAN_INTR_ENA_RBU | DMA_CHAN_INTR_ENA_RIE | \
	 DMA_CHAN_INTR_ENA_TIE)

/* DMA_CHAN_STATUS.NIS：Normal Interrupt Summary，正常中断汇总位，通常 W1C 清除。 */
#define DMA_CHAN_STATUS_NIS		(1U << 15)
/* DMA_CHAN_STATUS.AIS：Abnormal Interrupt Summary，异常中断汇总位，通常 W1C 清除。 */
#define DMA_CHAN_STATUS_AIS		(1U << 14)
/* DMA_CHAN_STATUS.RI：Receive Interrupt，RX 描述符完成/有帧写入。 */
#define DMA_CHAN_STATUS_RI		(1U << 6)
/* DMA_CHAN_STATUS.TI：Transmit Interrupt，TX 描述符完成/帧发送完成。 */
#define DMA_CHAN_STATUS_TI		(1U << 0)

/* DMA_HW_FEATURE0.ACTPHYIF：硬件实际综合的 PHY interface 类型字段，bit[30:28]。 */
#define DMA_HW_FEAT_ACTPHYIF_MASK	(7U << 28)

#endif /* EQOS_REGS_H */
