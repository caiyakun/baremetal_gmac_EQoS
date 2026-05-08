/* 缺省 MMIO 桩：在真实工程中删除本文件并提供 gmac_io_read32 / gmac_io_write32。 */
#include <stdint.h>
#include <stddef.h>

#ifndef EQOS_STUB_REG_PTR
#define EQOS_STUB_REG_PTR ((volatile uint32_t *)0)
#endif

__attribute__((weak)) uint32_t gmac_io_read32(uintptr_t addr)
{
	(void)addr;
	return *EQOS_STUB_REG_PTR;
}

__attribute__((weak)) void gmac_io_write32(uintptr_t addr, uint32_t v)
{
	(void)addr;
	(void)v;
}

__attribute__((weak)) void gmac_dma_cache_wb(const void *addr, size_t size)
{
	(void)addr;
	(void)size;
}

__attribute__((weak)) void gmac_dma_cache_invalidate(void *addr, size_t size)
{
	(void)addr;
	(void)size;
}

__attribute__((weak)) void intr_handler_set(int source, void (*isr_handle)(void *), void *param)
{
	(void)source;
	(void)isr_handle;
	(void)param;
}

__attribute__((weak)) void esprv_intc_int_set_priority(int source, int priority)
{
	(void)source;
	(void)priority;
}

__attribute__((weak)) void esprv_intc_int_enable(int source)
{
	(void)source;
}

__attribute__((weak)) void esprv_intc_int_disable(int source)
{
	(void)source;
}
