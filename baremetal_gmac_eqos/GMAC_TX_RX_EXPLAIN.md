# GMAC TX/RX 配置与收发原理说明

本文说明当前 `baremetal_gmac_eqos` 裸机 GMAC 代码中的 TX 发送配置、RX 接收配置，以及 descriptor、buffer、DMA、MTL、MAC 之间的关系。

可以把整个 GMAC 数据路径理解成：

```text
CPU 内存里的 buffer/descriptor
        |
        v
DMA 读写 descriptor + buffer
        |
        v
MTL FIFO / Queue
        |
        v
MAC
        |
        v
PHY/RGMII 或 MAC near-end loopback
```

## 1. 两个核心概念

**buffer** 是真正放以太网帧数据的内存，例如：

- `s_dma->tx_buf[i]`：第 `i` 个 TX 发送 buffer
- `s_dma->rx_buf[i]`：第 `i` 个 RX 接收 buffer

**descriptor** 是 DMA 看的“任务单”。它告诉 DMA：

- 帧数据 buffer 在哪里
- 这帧多长
- 当前 descriptor 是一帧的第一段还是最后一段
- descriptor 当前归 CPU 还是 DMA

当前代码使用 DWMAC4 normal descriptor，每个 descriptor 是 4 个 32-bit：

```c
struct eqos_dma_desc {
	uint32_t des0;
	uint32_t des1;
	uint32_t des2;
	uint32_t des3;
};
```

最重要的是 `OWN` 位：

```text
OWN = 0：descriptor 归 CPU，CPU 可以修改
OWN = 1：descriptor 归 DMA，CPU 不应该再改
```

## 2. DMA 内存组织

当前代码先准备静态内存池，再绑定到 `s_dma`：

```c
static gmac_eqos_dma_t *gmac_eqos_new_dma(void)
{
	s_eqos_dma.tx_desc = s_txd;
	s_eqos_dma.rx_desc = s_rxd;
	s_eqos_dma.tx_buf = s_txb;
	s_eqos_dma.rx_buf = s_rxb;
	s_eqos_dma.tx_desc_num = EQOS_TX_RING;
	s_eqos_dma.rx_desc_num = EQOS_RX_RING;
	s_eqos_dma.buf_size = EQOS_BUF_SZ;

	return &s_eqos_dma;
}
```

所以后面所有 TX/RX 都是：

```text
TX:
s_dma->tx_desc[i]  描述第 i 个 TX 任务
s_dma->tx_buf[i]   第 i 个 TX 数据 buffer

RX:
s_dma->rx_desc[i]  描述第 i 个 RX buffer
s_dma->rx_buf[i]   第 i 个 RX 接收 buffer
```

这和 P5 代码中的 `gmac_esp_dma->tx_buf[i] = g_tx_buffers[i]` 语义类似，只是当前裸机版不动态分配，直接绑定静态数组。

## 3. TX 初始化配置

TX 初始化主要做三件事。

### 3.1 配置 TX DMA 行为

代码在 `gmac_hal_init_dma_default()`：

```c
v = reg_rd(hal, DMA_CHAN_TX_CONTROL);
v &= ~DMA_CHAN_TX_CTRL_TXPBL_MASK;
v |= (pbl << 16) & DMA_CHAN_TX_CTRL_TXPBL_MASK;
v |= DMA_CONTROL_OSP;
v &= ~DMA_CONTROL_TSE;
reg_wr(hal, DMA_CHAN_TX_CONTROL, v);
```

含义：

- `TXPBL = pbl`：TX DMA 一次 burst 搬运多少数据
- `OSP = 1`：允许 DMA 处理第二个包，提高连续发送效率
- `TSE = 0`：不启用 TCP segmentation，当前只发普通以太网帧
- 此时还没有置 `ST`，所以 TX DMA 还没启动

### 3.2 清空 TX descriptor

代码在 `gmac_hal_init_desc_ring()`：

```c
for (unsigned i = 0; i < s_dma->tx_desc_num; i++) {
	memset(&s_dma->tx_desc[i], 0, sizeof(s_dma->tx_desc[i]));
	DMA_CACHE_WB(&s_dma->tx_desc[i], EQOS_DMA_DESC_SZ);
}
```

`memset(..., 0)` 之后，`TDES3_OWN` 是 0，所以 TX descriptor 归 CPU 所有，处于空闲可用状态。

`DMA_CACHE_WB()` 的作用是：CPU 刚清零 descriptor，必须把 cache 写回内存，DMA 后续才能看到最新状态。如果没有 D-cache，该函数是空实现。

### 3.3 告诉 DMA TX descriptor ring 在哪里

代码在 `gmac_hal_init_desc_ring()`：

```c
reg_wr(hal, DMA_CHAN_TX_BASE_ADDR_HI, 0);
reg_wr(hal, DMA_CHAN_TX_BASE_ADDR, EQOS_VIRT_TO_PHYS(s_dma->tx_desc));
reg_wr(hal, DMA_CHAN_TX_RING_LEN, s_dma->tx_desc_num - 1U);
reg_wr(hal, DMA_CHAN_TX_END_ADDR, EQOS_VIRT_TO_PHYS(&s_dma->tx_desc[0]));
```

含义：

- `DMA_CHAN_TX_BASE_ADDR`：TX descriptor ring 起始地址
- `DMA_CHAN_TX_RING_LEN`：ring 里 descriptor 数量减 1
- `DMA_CHAN_TX_END_ADDR`：TX tail pointer 初始值

TX tail 初始化到 `tx_desc[0]` 只是给硬件一个合法初始位置。真正触发发送的是 `gmac_mac_transmit()` 里再次写 tail。

## 4. TX 发送过程

发送函数是 `gmac_mac_transmit()`。

### 4.1 找一个 CPU 可用的 TX descriptor

```c
for (unsigned i = 0; i < s_dma->tx_desc_num; i++) {
	DMA_CACHE_INVALIDATE(&s_dma->tx_desc[i], EQOS_DMA_DESC_SZ);
	EQOS_MEM_BARRIER();
	d3 = s_dma->tx_desc[i].des3;
	if (!(d3 & TDES3_OWN)) {
		slot = i;
		break;
	}
}
```

这里先 `DMA_CACHE_INVALIDATE()`，因为 descriptor 可能刚被 DMA 改过。CPU 必须丢掉旧 cache，再看内存里的真实 `OWN` 位。

如果 `OWN = 0`，说明这个 descriptor 已经归 CPU，可以用于本次发送。

### 4.2 把用户数据复制到 TX buffer

```c
memcpy(s_dma->tx_buf[slot], buf, length);
DMA_CACHE_WB(s_dma->tx_buf[slot], s_dma->buf_size);
```

`s_dma->tx_buf[slot]` 就是第 `slot` 个 TX frame buffer。

`DMA_CACHE_WB(tx_buf)` 很关键：CPU 写了帧数据，DMA 要从内存读这个帧，所以必须写回 cache。

### 4.3 准备 TX descriptor

```c
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
```

含义：

- `des0 = buf_phys`：告诉 DMA 帧数据在哪里
- `des1 = 0`：buffer 地址高 32 位，当前默认 32-bit 地址空间
- `des2 = len`：buffer1 里有多少字节
- `des3 packet size`：整帧长度
- `TDES3_FIRST_DESCRIPTOR`：这是这一帧的第一个 descriptor
- `TDES3_LAST_DESCRIPTOR`：这是这一帧的最后一个 descriptor
- 当前一帧只使用一个 descriptor，所以同时设置 FIRST 和 LAST
- 最后置 `TDES3_OWN`：把 descriptor 交给 DMA
- 再 `DMA_CACHE_WB()`：让 DMA 看到 descriptor 最新内容

### 4.4 写 TX tail pointer 触发 DMA

```c
EQOS_MEM_BARRIER();
dma_prog_tx_tail(hal, slot);
```

`dma_prog_tx_tail()` 会写：

```c
DMA_CHAN_TX_END_ADDR = &s_dma->tx_desc[slot]
```

这相当于告诉 DMA：“我已经准备好了新的 TX descriptor，你可以来取了。”

### 4.5 等待 DMA 发送完成

```c
while (tries--) {
	DMA_CACHE_INVALIDATE(&s_dma->tx_desc[slot], EQOS_DMA_DESC_SZ);
	EQOS_MEM_BARRIER();
	d3 = s_dma->tx_desc[slot].des3;
	if (!(d3 & TDES3_OWN)) {
		if (d3 & TDES3_ERROR_SUMMARY)
			return -1;
		return 0;
	}
}
```

DMA 发送完成后会清 `OWN`。CPU 看到 `OWN=0` 就知道发送完成。

## 5. RX 初始化配置

RX 初始化和 TX 相反：

```text
TX descriptor 初始化时归 CPU
RX descriptor 初始化时直接交给 DMA
```

### 5.1 配置 RX DMA 行为

代码在 `gmac_hal_init_dma_default()`：

```c
v = reg_rd(hal, DMA_CHAN_RX_CONTROL);
v &= ~DMA_CHAN_RX_CTRL_RXPBL_MASK;
v |= (pbl << 16) & DMA_CHAN_RX_CTRL_RXPBL_MASK;
v &= ~DMA_RBSZ_MASK;
v |= (EQOS_BUF_SZ << 1) & DMA_RBSZ_MASK;
reg_wr(hal, DMA_CHAN_RX_CONTROL, v);
```

含义：

- `RXPBL = pbl`：RX DMA burst length
- `RBSZ`：RX buffer size，当前对应 `EQOS_BUF_SZ = 2048`
- 此时还没置 `SR`，RX DMA 还没启动

### 5.2 初始化每个 RX descriptor

代码在 `gmac_hal_init_desc_ring()`：

```c
for (unsigned i = 0; i < s_dma->rx_desc_num; i++)
	rx_desc_reset_owned(&s_dma->rx_desc[i],
			    EQOS_VIRT_TO_PHYS(s_dma->rx_buf[i]));
```

这句把 `rx_buf[i]` 绑定到 `rx_desc[i]`。

真正写 descriptor 的地方是 `rx_desc_reset_owned()`：

```c
d->des0 = buf_phys;
d->des1 = 0;
d->des2 = 0;
d->des3 = RDES3_BUFFER1_VALID_ADDR | RDES3_INT_ON_COMPLETION_EN;
EQOS_MEM_BARRIER();
d->des3 |= RDES3_OWN;
EQOS_MEM_BARRIER();
DMA_CACHE_WB(d, EQOS_DMA_DESC_SZ);
```

含义：

- `des0 = rx buffer 地址`
- `des1 = 0`：地址高 32 位
- `des2 = 0`：当前只用 buffer1，不使用 buffer2
- `RDES3_BUFFER1_VALID_ADDR`：告诉 DMA buffer1 地址有效
- `RDES3_INT_ON_COMPLETION_EN`：收完后可以产生 RX complete 状态
- `RDES3_OWN = 1`：这个 RX descriptor 交给 DMA
- `DMA_CACHE_WB()`：让 DMA 看见这些设置

### 5.3 告诉 DMA RX descriptor ring 在哪里

```c
reg_wr(hal, DMA_CHAN_RX_BASE_ADDR_HI, 0);
reg_wr(hal, DMA_CHAN_RX_BASE_ADDR, EQOS_VIRT_TO_PHYS(s_dma->rx_desc));
reg_wr(hal, DMA_CHAN_RX_RING_LEN, s_dma->rx_desc_num - 1U);
dma_prog_rx_tail_last(hal, s_dma->rx_desc_num - 1U);
```

含义：

- `DMA_CHAN_RX_BASE_ADDR`：RX descriptor ring 起点
- `DMA_CHAN_RX_RING_LEN`：RX descriptor 数量减 1
- `DMA_CHAN_RX_END_ADDR`：RX tail pointer，初始化指到最后一个 descriptor

这表示：`rx_desc[0]` 到 `rx_desc[last]` 都可以给 DMA 收包。

## 6. RX 接收过程

接收函数是 `gmac_receive_frame()`。

### 6.1 扫描 RX descriptor ring

```c
for (unsigned k = 0; k < s_dma->rx_desc_num; k++) {
	unsigned i = (s_rx_scan + k) % s_dma->rx_desc_num;
	struct eqos_dma_desc *d = &s_dma->rx_desc[i];

	DMA_CACHE_INVALIDATE(d, EQOS_DMA_DESC_SZ);
	EQOS_MEM_BARRIER();
	d3 = d->des3;
	if (d3 & RDES3_OWN)
		continue;
```

如果 `OWN=1`，说明 DMA 还没用完这个 descriptor，CPU 跳过。

如果 `OWN=0`，说明 DMA 已经写完了，CPU 可以读取状态和 buffer。

### 6.2 检查这是不是一个有效完整帧

```c
if (d3 & RDES3_CONTEXT_DESCRIPTOR)
	continue;
if (!(d3 & RDES3_LAST_DESCRIPTOR))
	continue;
if (d3 & RDES3_ERROR_SUMMARY)
	goto bad;
```

当前只处理“一帧一个 descriptor”的普通帧，所以要求 `LAST_DESCRIPTOR` 有效。

如果 `ERROR_SUMMARY` 置位，说明 DMA/MAC 在接收这帧时发现错误，丢弃并回收 descriptor。

### 6.3 取帧长度并拷贝数据

```c
unsigned flen = d3 & RDES3_PACKET_SIZE_MASK;

if (flen < GMAC_CRC_LENGTH)
	goto bad;
flen -= GMAC_CRC_LENGTH;
if (flen > s_dma->buf_size)
	flen = s_dma->buf_size;
DMA_CACHE_INVALIDATE(s_dma->rx_buf[i], s_dma->buf_size);
memcpy(tmp, s_dma->rx_buf[i], flen);
```

`DMA_CACHE_INVALIDATE(rx_buf)` 很关键：RX buffer 是 DMA 写的，CPU 要读之前必须让 cache 失效，否则可能读到旧数据。

这里减掉 `GMAC_CRC_LENGTH`，是因为硬件上报的长度通常包含 FCS/CRC，而上层比较 payload 时不希望把 CRC 算进去。

### 6.4 回收 RX descriptor

```c
rx_desc_reset_owned(d, EQOS_VIRT_TO_PHYS(s_dma->rx_buf[i]));
dma_prog_rx_tail_last(hal, i);
s_rx_scan = (i + 1U) % s_dma->rx_desc_num;
```

CPU 读完这包后，要重新设置：

- buffer 地址
- buffer valid
- interrupt enable
- `OWN=1`

然后写 RX tail pointer，告诉 DMA：“这个 descriptor 又可以用了。”

## 7. MTL 和 MAC 启动

DMA 只是负责内存搬运。DMA 和 MAC 之间还有 MTL queue。

当前配置 queue0：

```text
MTL TX queue: enable + threshold mode（Store-and-Forward disabled, TTC=64B）
MTL RX queue: threshold mode（Store-and-Forward disabled, RTC=64B）
RX queue0 -> DMA channel0
MAC RX queue0 enable
```

说明：

- TX `TSF=0`：关闭 Transmit Store-and-Forward，TX FIFO 不再等完整帧进入后才发送；当前设置 `TTC=64B`，表示 TX FIFO 达到约 64 字节阈值后即可开始往 MAC 发。
- RX `RSF=0`：关闭 Receive Store-and-Forward，RX FIFO 不再等完整帧接收完后才交给 DMA；当前设置 `RTC=64B`，表示 RX FIFO 达到约 64 字节阈值后即可触发 DMA 写 `rx_buf`。
- 这个配置对齐 P5 `gmac_hal_init_dma_default()` 中的 `EMAC_LL_TRANSMIT_THRESHOLD_CONTROL_64` / `EMAC_LL_RECEIVE_THRESHOLD_CONTROL_64` 思路。

最后启动顺序是：

```text
先启动 RX DMA
再启动 TX DMA
最后打开 MAC RE/TE
```

这样做是为了避免 MAC 已经开始收包/发包，但 DMA 还没准备好 descriptor。

## 8. MAC near-end loopback 下 TX/RX 如何连起来

测试里会打开：

```text
GMAC_CONFIG.LM = 1
```

这叫 MAC near-end loopback。

所以数据路径变成：

```text
CPU 准备 TX buffer
  -> DMA 读 TX descriptor
  -> DMA 读 TX buffer
  -> MTL TX FIFO
  -> MAC TX
  -> MAC 内部 loopback
  -> MAC RX
  -> MTL RX FIFO
  -> DMA 写 RX buffer
  -> CPU 读取 RX buffer 校验
```

这就是为什么当前测试可以不依赖真实网线，也能验证 MAC/DMA/descriptor/cache 这条路径。

## 9. TX 和 RX ownership 对比

TX 是“CPU 准备好后交给 DMA”：

```text
CPU 写 tx_buf
CPU 写 tx_desc
CPU 置 OWN=1
CPU 写 TX tail
DMA 发送
DMA 清 OWN
CPU 看到 OWN=0，发送完成
```

RX 是“CPU 先把空 buffer 交给 DMA”：

```text
CPU 写 rx_desc 指向 rx_buf
CPU 置 OWN=1
DMA 收包写 rx_buf
DMA 清 OWN
CPU 读 rx_buf
CPU 再次置 OWN=1 还给 DMA
```

所以 TX 和 RX 的 ownership 方向是反的：

```text
TX 初始化：OWN=0，CPU 等待要发时再交 DMA
RX 初始化：OWN=1，提前交给 DMA 等待收包
```

## 10. Cache 操作为什么重要

如果 CPU 有 D-cache，而 DMA 不是 cache coherent，那么 CPU 和 DMA 看到的内存可能不一致。

### CPU 写，DMA 读

例如 TX：

```text
CPU 写 tx_buf / tx_desc
DMA 读 tx_buf / tx_desc
```

这时要 `DMA_CACHE_WB()`，把 CPU cache 写回内存。

### DMA 写，CPU 读

例如 RX：

```text
DMA 写 rx_buf / rx_desc
CPU 读 rx_buf / rx_desc
```

这时要 `DMA_CACHE_INVALIDATE()`，让 CPU 丢掉旧 cache，从内存重新读 DMA 写的新内容。

### 当前代码中的典型位置

TX 发送前：

```c
DMA_CACHE_WB(s_dma->tx_buf[slot], s_dma->buf_size);
DMA_CACHE_WB(&s_dma->tx_desc[slot], EQOS_DMA_DESC_SZ);
```

TX 等待完成：

```c
DMA_CACHE_INVALIDATE(&s_dma->tx_desc[slot], EQOS_DMA_DESC_SZ);
```

RX 检查 descriptor：

```c
DMA_CACHE_INVALIDATE(d, EQOS_DMA_DESC_SZ);
```

RX 读取 buffer：

```c
DMA_CACHE_INVALIDATE(s_dma->rx_buf[i], s_dma->buf_size);
```

RX 重新交给 DMA：

```c
DMA_CACHE_WB(d, EQOS_DMA_DESC_SZ);
```

