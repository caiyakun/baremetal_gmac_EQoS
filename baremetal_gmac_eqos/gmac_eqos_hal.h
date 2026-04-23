/*
 * Synopsys EQoS 5.40a (DWMAC4) 裸机 HAL — MAC Near-end loopback 与参考 test_gmac.c 流程对齐。
 * 将本目录加入 include path，并在板级提供 gmac_io_read32 / gmac_io_write32。
 */
#ifndef GMAC_EQOS_HAL_H
#define GMAC_EQOS_HAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * D-cache 维护（与 P5 test_gmac.c 中 DMA_CACHE_WB / DMA_CACHE_INVALIDATE 语义一致）
 *
 * - 默认：宏展开为 gmac_dma_cache_wb / gmac_dma_cache_invalidate（weak 空实现，无 D-cache 时零开销）。
 * - 板级：在链接的 BSP 中提供强符号实现，例如调用 dcache_cpa_range / dcache_ipa_range（与 P5 相同）。
 * - 或：在 #include 本头文件之前自行 #define DMA_CACHE_WB / DMA_CACHE_INVALIDATE 覆盖整个工程。
 */
#ifndef DMA_CACHE_WB
void gmac_dma_cache_wb(const void *addr, size_t size);
void gmac_dma_cache_invalidate(void *addr, size_t size);
#define DMA_CACHE_WB(addr, size) \
	gmac_dma_cache_wb((const void *)(uintptr_t)(addr), (size_t)(size))
#define DMA_CACHE_INVALIDATE(addr, size) \
	gmac_dma_cache_invalidate((void *)(uintptr_t)(addr), (size_t)(size))
#endif

typedef enum {
	GMAC_SPEED_10M = 0,
	GMAC_SPEED_100M,
	GMAC_SPEED_1000M,
} gmac_speed_t;

typedef enum {
	GMAC_DUPLEX_HALF = 0,
	GMAC_DUPLEX_FULL,
} gmac_duplex_t;

typedef struct {
	uintptr_t csr_base; /* EQoS CSR 统一基址（含 MAC/MTL/DMA） */
	uint8_t mac_addr[6];
	/* 内部状态，勿直接修改 */
	uint32_t _priv[64];
} gmac_hal_context_t;

#define ETH_HEADER_LEN		14U
#define GMAC_CRC_LENGTH		4U
#define GMAC_MY_NORMAL_PKT_TYPE	0x88B5U /* 与参考工程一致，自定义 EtherType */
#define GMAC_MY_STOP_PKT_TYPE	0x55AAU

typedef struct __attribute__((packed)) {
	uint8_t dest[6];
	uint8_t src[6];
	uint16_t proto; /* 网络字节序 */
	uint8_t data[];
} gmac_frame_t;

#ifndef GMAC_HTONS
#define GMAC_HTONS(x) ((uint16_t)((((x) & 0xffU) << 8) | (((x) >> 8) & 0xffU)))
#endif
#ifndef GMAC_NTOHS
#define GMAC_NTOHS(x) GMAC_HTONS(x)
#endif

/* 板级 MMIO：必须在链接的工程里实现 */
extern uint32_t gmac_io_read32(uintptr_t addr);
extern void gmac_io_write32(uintptr_t addr, uint32_t v);

/* 可选：CSR 时钟 Hz，用于 1us tick；未知时可传 0 则跳过写 1US 计数器 */
void gmac_eqos_set_csr_clock_hz(unsigned hz);

void gmac_glb_cfg_init(gmac_hal_context_t *hal);
void gmac_phy_vcs8541_init(void *phy_cfg, gmac_hal_context_t *hal, void *phy_extra);
void gmac_phy_802_3_basic_phy_deinit(void *phy_cfg, gmac_hal_context_t *hal);

void gmac_mac_init(gmac_hal_context_t *hal);
void gmac_mac_near_loopback_prepare(gmac_hal_context_t *hal);
void gmac_mac_set_speed(gmac_hal_context_t *hal, gmac_speed_t speed);
void gmac_mac_set_duplex(gmac_hal_context_t *hal, gmac_duplex_t duplex);
void gmac_mac_start(gmac_hal_context_t *hal);
int gmac_mac_stop(gmac_hal_context_t *hal);
void gmac_mac_del(void);

int gmac_mac_transmit(gmac_hal_context_t *hal, const void *buf, uint32_t length);
bool gmac_get_receive_finish_int_flag(gmac_hal_context_t *hal);
bool gmac_receive_frame(gmac_hal_context_t *hal, uint16_t *expected_frame,
			bool check_content_flag, bool return_on_every_good_pkt,
			bool timestamp_print);

/* 参考 test_gmac.c 的入口：调用前必须先绑定 CSR 基址（与 AXI 从端口一致） */
void gmac_eqos_test_bind(uintptr_t csr_base, unsigned csr_clock_hz);
void test_mac_near_end_loopback_force_link(gmac_speed_t force_link_speed);

#endif /* GMAC_EQOS_HAL_H */
