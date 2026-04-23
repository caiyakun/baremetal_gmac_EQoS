/*
 * Synopsys EQoS 5.40a / DWMAC4 单通道裸机 HAL
 * 参考：Linux drivers/net/ethernet/stmicro/stmmac (dwmac4_*)
 *       以及 P5 test_gmac.c 中 test_mac_near_end_loopback_force_link 流程
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

static struct eqos_dma_desc s_txd[EQOS_TX_RING] __attribute__((aligned(64)));
static struct eqos_dma_desc s_rxd[EQOS_RX_RING] __attribute__((aligned(64)));
static uint8_t s_txb[EQOS_TX_RING][EQOS_BUF_SZ] __attribute__((aligned(32)));
static uint8_t s_rxb[EQOS_RX_RING][EQOS_BUF_SZ] __attribute__((aligned(32)));

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
	memset(d, 0, sizeof(*d));
	d->des0 = buf_phys;
	d->des1 = 0;
	d->des2 = 0;
	d->des3 = RDES3_BUFFER1_VALID_ADDR | RDES3_INT_ON_COMPLETION_EN;
	EQOS_MEM_BARRIER();
	d->des3 |= RDES3_OWN;
	EQOS_MEM_BARRIER();
	DMA_CACHE_WB(d, EQOS_DMA_DESC_SZ);
}

static void dma_prog_rx_tail_last(gmac_hal_context_t *h, unsigned last_idx)
{
	uint32_t a = EQOS_VIRT_TO_PHYS(&s_rxd[last_idx]);

	reg_wr(h, DMA_CHAN_RX_END_ADDR, a);
}

static void dma_prog_tx_tail(gmac_hal_context_t *h, unsigned idx)
{
	uint32_t a = EQOS_VIRT_TO_PHYS(&s_txd[idx]);

	reg_wr(h, DMA_CHAN_TX_END_ADDR, a);
}

static void mtl_sf_mac_queues(gmac_hal_context_t *h)
{
	uint32_t v;

	v = reg_rd(h, MTL_CHAN_TX_OP_MODE);
	v &= ~MTL_OP_MODE_TXQEN_MASK;
	v |= MTL_OP_MODE_TXQEN | MTL_OP_MODE_TSF | MTL_OP_MODE_DIS_TCP_EF;
	reg_wr(h, MTL_CHAN_TX_OP_MODE, v);

	v = reg_rd(h, MTL_CHAN_RX_OP_MODE);
	v |= MTL_OP_MODE_RSF | MTL_OP_MODE_DIS_TCP_EF;
	reg_wr(h, MTL_CHAN_RX_OP_MODE, v);

	v = reg_rd(h, MTL_RXQ_DMA_MAP0);
	v &= ~MTL_RXQ_DMA_QXMDMACH_MASK(0);
	v |= MTL_RXQ_DMA_QXMDMACH(0, 0);
	reg_wr(h, MTL_RXQ_DMA_MAP0, v);

	v = reg_rd(h, GMAC_RXQ_CTRL0);
	v &= GMAC_RX_QUEUE_CLEAR(0);
	v |= GMAC_RX_DCB_QUEUE_ENABLE(0);
	reg_wr(h, GMAC_RXQ_CTRL0, v);
}

static void mac_set_addr0(gmac_hal_context_t *h, const uint8_t *mac)
{
	uint32_t hi = ((uint32_t)mac[5] << 8) | mac[4];

	hi |= (0U << GMAC_HI_DCS_SHIFT) & GMAC_HI_DCS_MASK;
	reg_wr(h, GMAC_ADDR_HIGH(0), hi | GMAC_HI_REG_AE);
	reg_wr(h, GMAC_ADDR_LOW(0),
	       ((uint32_t)mac[3] << 24) | ((uint32_t)mac[2] << 16) |
		       ((uint32_t)mac[1] << 8) | mac[0]);
	memcpy(h->mac_addr, mac, 6);
}

void gmac_glb_cfg_init(gmac_hal_context_t *hal)
{
	(void)hal;
}

void gmac_phy_vcs8541_init(void *phy_cfg, gmac_hal_context_t *hal, void *phy_extra)
{
	(void)phy_cfg;
	(void)hal;
	(void)phy_extra;
}

void gmac_phy_802_3_basic_phy_deinit(void *phy_cfg, gmac_hal_context_t *hal)
{
	(void)phy_cfg;
	(void)hal;
}

void gmac_mac_init(gmac_hal_context_t *hal)
{
	uint32_t v;

	s_hal = hal;
	s_rx_scan = 0;

	if (dma_soft_reset(hal) != 0)
		GMAC_PRINTF("EQoS: DMA soft reset timeout\n");

	/* 总线模式：对齐突发 + 固定 burst（与 dwc-qos 常用配置一致） */
	v = reg_rd(hal, DMA_SYS_BUS_MODE);
	v |= DMA_SYS_BUS_AAL | DMA_SYS_BUS_FB;
	reg_wr(hal, DMA_SYS_BUS_MODE, v);

	v = reg_rd(hal, DMA_BUS_MODE);
	v &= ~DMA_BUS_MODE_DCHE;
	reg_wr(hal, DMA_BUS_MODE, v);

	/* 通道控制 */
	v = reg_rd(hal, DMA_CHAN_CONTROL);
	v &= ~DMA_CHAN_CTRL_PBLX8;
	reg_wr(hal, DMA_CHAN_CONTROL, v);

	reg_wr(hal, DMA_CHAN_INTR_ENA, DMA_CHAN_INTR_DEFAULT_MASK_4_10);

	/* RX PBL + 接收缓冲长度（字节写入 RBSZ 域，stmmac/dwmac4 路径） */
	v = reg_rd(hal, DMA_CHAN_RX_CONTROL);
	v &= ~DMA_CHAN_RX_CTRL_RXPBL_MASK;
	v |= (PBL_VAL << 16) & DMA_CHAN_RX_CTRL_RXPBL_MASK;
	v &= ~DMA_RBSZ_MASK;
	v |= (EQOS_BUF_SZ << 1) & DMA_RBSZ_MASK;
	reg_wr(hal, DMA_CHAN_RX_CONTROL, v);

	v = reg_rd(hal, DMA_CHAN_TX_CONTROL);
	v &= ~DMA_CHAN_TX_CTRL_TXPBL_MASK;
	v |= (PBL_VAL << 16) & DMA_CHAN_TX_CTRL_TXPBL_MASK;
	v |= DMA_CONTROL_OSP;
	v &= ~DMA_CONTROL_TSE;
	reg_wr(hal, DMA_CHAN_TX_CONTROL, v);

	/* 描述符环 */
	for (unsigned i = 0; i < EQOS_TX_RING; i++) {
		memset(&s_txd[i], 0, sizeof(s_txd[i]));
		DMA_CACHE_WB(&s_txd[i], EQOS_DMA_DESC_SZ);
	}
	for (unsigned i = 0; i < EQOS_RX_RING; i++) {
		rx_desc_reset_owned(&s_rxd[i],
				    EQOS_VIRT_TO_PHYS(s_rxb[i]));
	}

	reg_wr(hal, DMA_CHAN_RX_BASE_ADDR_HI, 0);
	reg_wr(hal, DMA_CHAN_RX_BASE_ADDR, EQOS_VIRT_TO_PHYS(s_rxd));
	reg_wr(hal, DMA_CHAN_TX_BASE_ADDR_HI, 0);
	reg_wr(hal, DMA_CHAN_TX_BASE_ADDR, EQOS_VIRT_TO_PHYS(s_txd));

	reg_wr(hal, DMA_CHAN_RX_RING_LEN, EQOS_RX_RING - 1U);
	reg_wr(hal, DMA_CHAN_TX_RING_LEN, EQOS_TX_RING - 1U);

	mtl_sf_mac_queues(hal);

	/* MAC 基础：与 GMAC_CORE_INIT 同类位 + IPC */
	v = reg_rd(hal, GMAC_CONFIG);
	v |= GMAC_CONFIG_JD | GMAC_CONFIG_JE | GMAC_CONFIG_BE |
	     GMAC_CONFIG_DCRS | GMAC_CONFIG_IPC;
	v &= ~(GMAC_CONFIG_TE | GMAC_CONFIG_RE | GMAC_CONFIG_LM |
	       GMAC_CONFIG_PS | GMAC_CONFIG_FES);
	reg_wr(hal, GMAC_CONFIG, v);

	if (s_csr_hz >= 1000000U) {
		reg_wr(hal, GMAC4_MAC_ONEUS_TIC_COUNTER,
		       (s_csr_hz / 1000000U) - 1U);
	}

	const uint8_t defmac[] = { MAC_TEST_SRC_ADDR };

	mac_set_addr0(hal, defmac);

	/* RGMII：部分集成在 Near-end loopback 下仍要求 LUD 置位 */
	v = reg_rd(hal, GMAC_PHYIF_CONTROL_STATUS);
	v |= GMAC_PHYIF_CTRLSTATUS_LUD;
	reg_wr(hal, GMAC_PHYIF_CONTROL_STATUS, v);

	dma_prog_rx_tail_last(hal, EQOS_RX_RING - 1U);
	reg_wr(hal, DMA_CHAN_TX_END_ADDR, EQOS_VIRT_TO_PHYS(&s_txd[0]));

	GMAC_PRINTF("EQoS: mac_init done, VERSION=0x%08x\n",
		    (unsigned)reg_rd(hal, GMAC4_VERSION));
}

void gmac_mac_near_loopback_prepare(gmac_hal_context_t *hal)
{
	uint32_t v;

	v = reg_rd(hal, GMAC_PACKET_FILTER);
	v &= ~(GMAC_PACKET_FILTER_PR | GMAC_PACKET_FILTER_PM |
	       GMAC_PACKET_FILTER_RA);
	v |= GMAC_PACKET_FILTER_PR | GMAC_PACKET_FILTER_RA;
	reg_wr(hal, GMAC_PACKET_FILTER, v);

	v = reg_rd(hal, GMAC_CONFIG);
	v |= GMAC_CONFIG_LM;
	reg_wr(hal, GMAC_CONFIG, v);
}

void gmac_mac_set_speed(gmac_hal_context_t *hal, gmac_speed_t speed)
{
	uint32_t v = reg_rd(hal, GMAC_CONFIG);

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

	if (duplex == GMAC_DUPLEX_FULL)
		v |= GMAC_CONFIG_DM;
	else
		v &= ~GMAC_CONFIG_DM;
	reg_wr(hal, GMAC_CONFIG, v);
}

void gmac_mac_start(gmac_hal_context_t *hal)
{
	uint32_t v;

	v = reg_rd(hal, DMA_CHAN_RX_CONTROL);
	v |= DMA_CONTROL_SR;
	reg_wr(hal, DMA_CHAN_RX_CONTROL, v);

	v = reg_rd(hal, DMA_CHAN_TX_CONTROL);
	v |= DMA_CONTROL_ST;
	reg_wr(hal, DMA_CHAN_TX_CONTROL, v);

	v = reg_rd(hal, GMAC_CONFIG);
	v |= GMAC_CONFIG_RE | GMAC_CONFIG_TE;
	reg_wr(hal, GMAC_CONFIG, v);
}

int gmac_mac_stop(gmac_hal_context_t *hal)
{
	uint32_t v;

	v = reg_rd(hal, DMA_CHAN_TX_CONTROL);
	v &= ~DMA_CONTROL_ST;
	reg_wr(hal, DMA_CHAN_TX_CONTROL, v);

	v = reg_rd(hal, DMA_CHAN_RX_CONTROL);
	v &= ~DMA_CONTROL_SR;
	reg_wr(hal, DMA_CHAN_RX_CONTROL, v);

	v = reg_rd(hal, GMAC_CONFIG);
	v &= ~(GMAC_CONFIG_TE | GMAC_CONFIG_RE);
	reg_wr(hal, GMAC_CONFIG, v);

	return 0;
}

void gmac_mac_del(void)
{
	if (!s_hal)
		return;
	gmac_mac_stop(s_hal);
	{
		uint32_t v = reg_rd(s_hal, GMAC_CONFIG);

		v &= ~GMAC_CONFIG_LM;
		reg_wr(s_hal, GMAC_CONFIG, v);
	}
	s_hal = NULL;
}

int gmac_mac_transmit(gmac_hal_context_t *hal, const void *buf, uint32_t length)
{
	unsigned tries = 1000000U;
	unsigned slot = EQOS_TX_RING;

	if (!hal || length > EQOS_BUF_SZ || length < 16U)
		return -1;

	for (unsigned i = 0; i < EQOS_TX_RING; i++) {
		uint32_t d3;

		DMA_CACHE_INVALIDATE(&s_txd[i], EQOS_DMA_DESC_SZ);
		EQOS_MEM_BARRIER();
		d3 = s_txd[i].des3;
		if (!(d3 & TDES3_OWN)) {
			slot = i;
			break;
		}
	}
	if (slot >= EQOS_TX_RING)
		return -1;

	memcpy(s_txb[slot], buf, length);
	DMA_CACHE_WB(s_txb[slot], EQOS_BUF_SZ);
	tx_desc_prepare(&s_txd[slot], EQOS_VIRT_TO_PHYS(s_txb[slot]),
			length, length);
	EQOS_MEM_BARRIER();
	dma_prog_tx_tail(hal, slot);

	while (tries--) {
		uint32_t d3;

		DMA_CACHE_INVALIDATE(&s_txd[slot], EQOS_DMA_DESC_SZ);
		EQOS_MEM_BARRIER();
		d3 = s_txd[slot].des3;
		if (!(d3 & TDES3_OWN)) {
			if (d3 & TDES3_ERROR_SUMMARY)
				return -1;
			return 0;
		}
	}
	return -1;
}

bool gmac_get_receive_finish_int_flag(gmac_hal_context_t *hal)
{
	uint32_t st = reg_rd(hal, DMA_CHAN_STATUS);
	uint32_t en = reg_rd(hal, DMA_CHAN_INTR_ENA);

	if (st & DMA_CHAN_STATUS_RI) {
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

	for (unsigned k = 0; k < EQOS_RX_RING; k++) {
		unsigned i = (s_rx_scan + k) % EQOS_RX_RING;
		struct eqos_dma_desc *d = &s_rxd[i];
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
		if (d3 & RDES3_ERROR_SUMMARY)
			goto bad;

		{
			unsigned flen = d3 & RDES3_PACKET_SIZE_MASK;

			if (flen < GMAC_CRC_LENGTH)
				goto bad;
			flen -= GMAC_CRC_LENGTH;
			if (flen > EQOS_BUF_SZ)
				flen = EQOS_BUF_SZ;
			DMA_CACHE_INVALIDATE(s_rxb[i], EQOS_BUF_SZ);
			memcpy(tmp, s_rxb[i], flen);

			rx_desc_reset_owned(d, EQOS_VIRT_TO_PHYS(s_rxb[i]));
			dma_prog_rx_tail_last(hal, i);
			s_rx_scan = (i + 1U) % EQOS_RX_RING;

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
		rx_desc_reset_owned(d, EQOS_VIRT_TO_PHYS(s_rxb[i]));
		dma_prog_rx_tail_last(hal, i);
		s_rx_scan = (i + 1U) % EQOS_RX_RING;
	}
	return false;
}

void gmac_eqos_test_bind(uintptr_t csr_base, unsigned csr_clock_hz)
{
	s_test_csr = csr_base;
	s_test_csr_hz = csr_clock_hz;
}

void test_mac_near_end_loopback_force_link(gmac_speed_t force_link_speed)
{
	static gmac_hal_context_t ctx;
	const uint16_t test_len_max = 1514U;
	static const uint16_t test_len[] = {64, 128, 256, 512, 1024, 1514};
	uint8_t *pkt = (uint8_t *)s_txb[0];

	ctx.csr_base = s_test_csr;
	if (ctx.csr_base == 0) {
		GMAC_PRINTF("EQoS: call gmac_eqos_test_bind(csr_base, csr_hz) first\n");
		return;
	}
	gmac_eqos_set_csr_clock_hz(s_test_csr_hz);

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

	gmac_glb_cfg_init(&ctx);
	gmac_phy_vcs8541_init(NULL, &ctx, NULL);
	gmac_mac_init(&ctx);
	gmac_mac_near_loopback_prepare(&ctx);
	gmac_mac_set_speed(&ctx, force_link_speed);
	gmac_mac_set_duplex(&ctx, GMAC_DUPLEX_FULL);
	gmac_mac_start(&ctx);

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

	gmac_mac_stop(&ctx);
	gmac_phy_802_3_basic_phy_deinit(NULL, &ctx);
	gmac_mac_del();
}
