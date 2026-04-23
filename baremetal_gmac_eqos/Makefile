# 语法检查 / 主机侧空链接（无有效 MMIO）
CFLAGS ?= -Wall -Wextra -std=c11 -O2 -I.

OBJS = gmac_eqos_hal.o gmac_eqos_phy.o gmac_eqos_board_stub.o

.PHONY: all clean check
all: libgmac_eqos.a

libgmac_eqos.a: $(OBJS)
	$(AR) rcs $@ $(OBJS)

%.o: %.c gmac_eqos_hal.h gmac_eqos_phy.h eqos_regs.h eqos_desc.h
	$(CC) $(CFLAGS) -c $< -o $@

check: $(OBJS)
	@echo "OK"

clean:
	rm -f $(OBJS) libgmac_eqos.a
