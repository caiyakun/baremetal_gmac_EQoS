/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * 裸机测试侧「帧格式 + HAL 上下文」声明，与 gmac_eqos_compat.c 配套。
 *
 * 注意：
 *   - gmac_frame_t 仅用于自测协议解析；以太网 on-wire 为网络字节序，
 *     proto 字段需用 GMAC_NTOHS 比较常量。
 *   - expected_frame / check_content_flag / retrun_on_every_good_pkt 等参数
 *     沿用历史命名（含拼写），行为以 gmac_receive_frame 实现为准。
 */
#ifndef GMAC_EQOS_HAL_H
#define GMAC_EQOS_HAL_H

#include <stdint.h>
#include <stdbool.h>

#include "eqos_mdio.h"

/* 与对端/自测程序约定的 ether type，非 IANA 标准值，避免与 IPv4/ARP 冲突。 */
#define GMAC_MY_NORMAL_PKT_TYPE		0x2233u
#define GMAC_MY_STOP_PKT_TYPE		0x55aau
#define ETH_HEADER_LEN			14u

#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
#define GMAC_NTOHS(x) ((uint16_t)(x))
#define GMAC_HTONS(x) ((uint16_t)(x))
#else
#define GMAC_NTOHS(x) ((uint16_t)__builtin_bswap16((uint16_t)(x)))
#define GMAC_HTONS(x) ((uint16_t)__builtin_bswap16((uint16_t)(x)))
#endif

/* 最小以太帧头 + 变长 payload；data[] 为柔性数组成员，访问前须保证 pl
 * 已由描述符或长度检查约束。 */
typedef struct {
	uint8_t da[6];
	uint8_t sa[6];
	uint16_t proto;
	uint8_t data[];
} gmac_frame_t;

/* csr_base：MAC CSR 窗口基址（已含 SoC 层偏移则整段基址即可）。
 * mdio：与 MAC 内嵌 MDIO 主机共用 csr_base；csr_clk_hz 供分频计算。 */
typedef struct gmac_hal_context {
	uintptr_t csr_base;
	struct eqos_mdio_cfg mdio;
} gmac_hal_context_t;

struct gmac_esp_dma_state;

typedef struct gmac_esp_dma_state *gmac_esp_dma_handle_t;

void gmac_esp_new_dma(gmac_esp_dma_handle_t *out_handle);
void gmac_mac_init(gmac_hal_context_t *hal);

/* 返回实际发送字节数；0 表示环满、参数非法、或 DMA 未完成（超时）。 */
int32_t gmac_esp_dma_transmit_frame(gmac_esp_dma_handle_t dma, uint8_t *buf,
				    uint32_t length, gmac_hal_context_t *hal);

/* 无包可读返回 false；业务「成功」由 ok 路径与 retrun_on_every_good_pkt
 * 组合决定。无论是否 ok，成功走 done_rx 都会回收 Rx 描述符。 */
bool gmac_receive_frame(gmac_hal_context_t *hal, uint16_t *expected_frame,
			bool check_content_flag, bool retrun_on_every_good_pkt,
			bool timestamp_print);

#endif /* GMAC_EQOS_HAL_H */
