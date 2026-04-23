/* DWMAC4 正常描述符 (read / write-back)，与 Linux dwmac4_descs.h 一致。 */
#ifndef EQOS_DESC_H
#define EQOS_DESC_H

#include <stdint.h>

struct eqos_dma_desc {
	uint32_t des0;
	uint32_t des1;
	uint32_t des2;
	uint32_t des3;
};

#define TDES2_BUFFER1_SIZE_MASK		0x3fffU
#define TDES2_INTERRUPT_ON_COMPLETION	(1U << 31)

#define TDES3_PACKET_SIZE_MASK		0x7fffU
#define TDES3_CHECKSUM_INSERTION_MASK	(3U << 16)
#define TDES3_FIRST_DESCRIPTOR		(1U << 29)
#define TDES3_LAST_DESCRIPTOR		(1U << 28)
#define TDES3_ERROR_SUMMARY		(1U << 15)
#define TDES3_OWN			(1U << 31)

#define RDES3_OWN			(1U << 31)
#define RDES3_BUFFER1_VALID_ADDR	(1U << 24)
#define RDES3_INT_ON_COMPLETION_EN	(1U << 30)
#define RDES3_FIRST_DESCRIPTOR		(1U << 29)
#define RDES3_LAST_DESCRIPTOR		(1U << 28)
#define RDES3_ERROR_SUMMARY		(1U << 15)
#define RDES3_PACKET_SIZE_MASK		0x7fffU
#define RDES3_CONTEXT_DESCRIPTOR	(1U << 30)

#endif /* EQOS_DESC_H */
