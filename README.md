# CAI_TEST 工程说明（GMAC / EQoS 相关）

本文档整理 **Synopsys Controller IP Ethernet Quality-of-Service（EQoS）5.40a** 在 SoC 验证中的背景说明、Linux 驱动参考位置，以及本仓库内 **裸机 MAC Near-end Loopback** 参考实现的使用方式。后续若补充寄存器级细节、SoC 专用适配或测试结果，建议在本文件追加章节并保持日期记录。

---

## 1. 背景与目标

- **SoC**：自研芯片内集成 **GMAC IP**，版本为 **Synopsys Ethernet QoS 5.40a**（与 Databook 5.40a / Feb 2024 一致）。
- **验证环境**：FPGA 平台裸机（C 语言），接口模式 **RGMII**；先验证 **MAC Near-end Loopback**（近端环回），再扩展外接 PHY / 对端联调。
- **参考裸机流程**：逻辑与路径  
  `/satassd/01_FW/10-fpga_test/P5/02-direct_boot_code/gmac_test_prepare_p5_fpga/example/components/gmac/test/test_gmac.c`  
  中的 **`test_mac_near_end_loopback_force_link(gmac_speed_t force_link_speed)`** 对齐（初始化 → 环回准备 → 设速/双工 → 启动 → 多档长度发收 → 载荷比对）。
- **说明**：P5 工程里的 `test_gmac.c` 针对的是 **另一套 EMAC/GMAC 实现**，仅作**流程与测试方法**参考；本仓库 `baremetal_gmac_eqos` 中的代码按 **EQoS 5.40a / DWMAC4** 寄存器与描述符模型编写。

---

## 2. Linux 源码中的驱动参考（本仓库 `linux/` 目录）

### 2.1 路径与内核版本

- **树内路径**：`/satassd/01_FW/33-embed_linux/cai_test/linux`
- **内核版本**（Makefile）：**Linux 7.0.0**
- **主要驱动目录**：`drivers/net/ethernet/stmicro/stmmac/`
  - Synopsys **DWC Ethernet QoS** 平台绑定示例：`dwmac-dwc-qos-eth.c`（文件头仍保留早期 “v4.10a” 注释，实际已与主线 stmmac 一起演进）。
  - **MAC / DMA / 描述符**：`dwmac4_core.c`、`dwmac4_dma.c`、`dwmac4_lib.c`、`dwmac4_descs.c`、`dwmac4.h`、`dwmac4_dma.h` 等。
  - **设备树 compatible**：`stmmac_platform.c` 中 `stmmac_gmac4_compats[]` 已包含 **`snps,dwmac-5.40a`**。

### 2.1.1 `snps,dwmac-5.40a` 在 Linux stmmac 中的源码位置

完整路径：

`/satassd/01_FW/33-embed_linux/cai_test/linux/drivers/net/ethernet/stmicro/stmmac/stmmac_platform.c`

关键代码 1：`stmmac_gmac4_compats[]` 里已经列出 `snps,dwmac-5.40a`，说明 Linux stmmac 将 5.40a 按 **GMAC4 / DWMAC4 系列**处理。

```c
/* Compatible string array for all gmac4 devices */
static const char * const stmmac_gmac4_compats[] = {
	"snps,dwmac-4.00",
	"snps,dwmac-4.10a",
	"snps,dwmac-4.20a",
	"snps,dwmac-5.00a",
	"snps,dwmac-5.10a",
	"snps,dwmac-5.20",
	"snps,dwmac-5.30a",
	"snps,dwmac-5.40a",
	NULL
};
```

关键代码 2：平台解析设备树时，如果 compatible 匹配该数组，就把 core type 设置为 `DWMAC_CORE_GMAC4`，后续会走 `dwmac4_*` 系列 MAC/DMA/descriptor 实现。

```c
if (of_device_compatible_match(np, stmmac_gmac4_compats)) {
	plat->core_type = DWMAC_CORE_GMAC4;
	plat->pmt = true;
	if (of_property_read_bool(np, "snps,tso"))
		plat->flags |= STMMAC_FLAG_TSO_EN;
}
```

因此本仓库裸机实现选择参考这些 Linux 文件：

- `linux/drivers/net/ethernet/stmicro/stmmac/dwmac4_core.c`：MAC 配置、`GMAC_CONFIG`、MAC loopback 等。
- `linux/drivers/net/ethernet/stmicro/stmmac/dwmac4_dma.c`：DMA channel、PBL、ring base/tail、启动停止等。
- `linux/drivers/net/ethernet/stmicro/stmmac/dwmac4_descs.c`：DWMAC4 normal descriptor 格式与 TX/RX descriptor 读写。
- `linux/drivers/net/ethernet/stmicro/stmmac/dwmac4.h`：GMAC/MTL 寄存器 offset 与 bit。
- `linux/drivers/net/ethernet/stmicro/stmmac/dwmac4_dma.h`：DMA common/channel 寄存器 offset 与 bit。
- `linux/drivers/net/ethernet/stmicro/stmmac/stmmac_mdio.c`：GMAC4 MDIO `GMAC_MDIO_ADDR` / `GMAC_MDIO_DATA` 的读写格式。

### 2.2 与 5.40a 的适用性（摘要）

- **适用**：寄存器布局、DMA 单通道典型流程、MAC `LM` 环回、RGMII 相关 `GMAC_PHYIF_CONTROL_STATUS` 等与 **DWMAC4 / EQoS 同谱系** 的代码，均可作为裸机编程的主要参考。
- **需自行核对**：Databook 5.40a 中新增/细化的行为（例如描述符 **OWN=0 时 re-fetch**、接收 idle / 中断看门狗等），内核未必逐条显式对应；建议读 **`GMAC4_VERSION` / SNPSVER** 与手册对照，并在启用相关特性时对照 **Chapter 20 Programming** 与描述符章节。
- **环回与自测**：MAC 环回相关可重点看 `dwmac4_core.c` 中 **`dwmac4_set_mac_loopback()`**（`GMAC_CONFIG_LM`），以及 **`stmmac_selftests.c`** 中的 **`stmmac_test_mac_loopback()`**（依赖 netdev，不可直接搬裸机，但流程可参考）。

### 2.3 修改范围约定

- 本 **CAI_TEST** 仓库约定：**除 `linux/` 目录外** 可自行增删改；**请勿在交付流程中擅自改动 `linux/`** 下的上游树，除非团队另有规范。

---

## 3. 裸机参考实现：`baremetal_gmac_eqos/`

目录：`/satassd/01_FW/33-embed_linux/cai_test/baremetal_gmac_eqos/`

目的：在 **无操作系统** 条件下，提供与 **EQoS 5.40a（DWMAC4 单通道默认 CSR 布局）** 一致的 **DMA 环 + MAC Near-end Loopback** 最小通路，便于 FPGA 上先做 **数据链路层** 自发自收与载荷比对（不跑上层协议）。

### 3.1 文件清单

| 文件 | 说明 |
|------|------|
| `eqos_regs.h` | MAC / MTL / DMA CSR 偏移与常用位（对齐 Linux `dwmac4.h`、`dwmac4_dma.h`） |
| `eqos_desc.h` | DWMAC4 正常描述符结构与位定义（对齐 `dwmac4_descs.h`） |
| `gmac_eqos_hal.h` | 对外 API、帧类型与测试入口声明 |
| `gmac_eqos_hal.c` | `gmac_mac_init`、`gmac_mac_transmit`、`gmac_receive_frame`、DMA/MAC/MTL 初始化、环回与参考测试 `test_mac_near_end_loopback_force_link` |
| `gmac_eqos_board_stub.c` | **弱符号** 缺省 `gmac_io_read32` / `gmac_io_write32` 及 **`gmac_dma_cache_wb` / `gmac_dma_cache_invalidate`**，便于本目录 `make` 通过；真板请用实现覆盖 |
| `Makefile` | `make check` 编译检查；`make` 生成 `libgmac_eqos.a` |

### 3.2 主要 API（与参考流程对应）

| API | 作用 |
|-----|------|
| `gmac_eqos_test_bind(csr_base, csr_clock_hz)` | 设置 CSR 物理基址与 CSR 时钟（Hz）；`csr_clock_hz` 可为 0 则跳过 1us 计数器配置 |
| `gmac_mac_init(hal)` | DMA 软复位、总线模式、单通道 TX/RX 描述符环、MTL SF、RX 队列路由、MAC 基础位、MAC 地址 0、PHYIF `LUD` 等 |
| `gmac_mac_near_loopback_prepare(hal)` | 报文过滤（PR+RA）+ `GMAC_CONFIG_LM` |
| `gmac_mac_set_speed` / `gmac_mac_set_duplex` | `GMAC_CONFIG` 的 PS/FES/DM |
| `gmac_mac_start` / `gmac_mac_stop` | DMA SR/ST + MAC TE/RE |
| `gmac_mac_transmit` | 填充 TX 描述符并写 `DMA_CHAN_TX_END_ADDR`（尾指针），轮询 OWN 完成 |
| `gmac_get_receive_finish_int_flag` | 读/清 `DMA_CHAN_STATUS` 的 RI 等（可与轮询描述符配合） |
| `gmac_receive_frame` | 扫描 RX 环，取最后一节描述符长度（减 CRC），可选整帧载荷与 golden 比对 |
| `gmac_mac_del` | 停 DMA/MAC 并清环回 |
| `test_mac_near_end_loopback_force_link(speed)` | 与参考 `test_gmac.c` 相同的多档长度与 EtherType **`0x88B5`**、载荷 **`i & 0xff`** 比对 |

PHY 占位（与参考工程同名，便于替换）：`gmac_glb_cfg_init`、`gmac_phy_vcs8541_init`、`gmac_phy_802_3_basic_phy_deinit` 在本参考中为空实现，可按 SoC 接真实 PHY 驱动。

### 3.3 板级必做项

1. **实现** `uint32_t gmac_io_read32(uintptr_t addr)` 与 `void gmac_io_write32(uintptr_t addr, uint32_t v)`（或通过链接覆盖 `gmac_eqos_board_stub.c` 的 weak 符号），访问 **`hal->csr_base + offset`**。
2. **D-Cache（已实现与 P5 同名的宏与调用点）**  
   - 头文件 `gmac_eqos_hal.h` 中提供 **`DMA_CACHE_WB(addr, size)`**、**`DMA_CACHE_INVALIDATE(addr, size)`**，默认展开为 **`gmac_dma_cache_wb`** / **`gmac_dma_cache_invalidate`**（`gmac_eqos_board_stub.c` 内为 **weak 空实现**）。  
   - `gmac_eqos_hal.c` 已在关键路径调用：TX 缓冲写后 WB、TX 描述符提交后 WB、轮询 TX 完成前对描述符 INV、RX 读描述符前 INV、从 RX 缓冲拷出前对整行缓冲 INV、RX 描述符交还 DMA 前在 `rx_desc_reset_owned()` 内 WB 等（与 P5 `test_gmac.c` 中围绕描述符与 `Buffer1Addr` 的用法一致）。  
   - **板级实现**：在 BSP 中提供强符号，例如与 P5 相同调用 `dcache_cpa_range` / `dcache_ipa_range`（见  
     `.../gmac_test_prepare_p5_fpga/example/components/gmac/test/test_gmac.c` 中 `DMA_CACHE_WB` / `DMA_CACHE_INVALIDATE` 宏定义）。  
   - **或**：在 `#include "gmac_eqos_hal.h"` **之前**自行 `#define DMA_CACHE_WB` / `DMA_CACHE_INVALIDATE`，则不会使用默认的 `gmac_dma_cache_*` 声明。
3. **`EQOS_VIRT_TO_PHYS`**：默认 `(uint32_t)(uintptr_t)(p)`；若 DRAM 与设备可见地址不一致，请在编译前定义正确的物理地址转换宏。

### 3.4 编译

```bash
cd /satassd/01_FW/33-embed_linux/cai_test/baremetal_gmac_eqos
make check    # 仅编译 .o
make          # 生成 libgmac_eqos.a
```

### 3.5 调用示例（逻辑顺序）

```c
#include "gmac_eqos_hal.h"

void board_gmac_test(void)
{
    gmac_eqos_test_bind(0xYOUR_CSR_BASE_PHYS, 125000000u); /* 示例：125MHz CSR */
    test_mac_near_end_loopback_force_link(GMAC_SPEED_1000M);
}
```

### 3.6 限制与集成时注意

- 当前实现假定 **单 RX / 单 TX 队列**、**默认 CSR 偏移**（未使用 Linux 中 `dwmac4_addrs` 类平台覆盖）；SoC 若对 MAC/MTL/DMA 基址做 **窗口折叠或别名映射**，需改 `eqos_regs.h` 或增加一层 `reg_base` 分块。
- **`DMA_CHAN_RX_CONTROL` 中 RBSZ 域** 的编码与 Linux `stmmac_set_dma_bfsize` 路径一致；若你方 Databook 要求 **按半字/字对齐的另一种编码**，请按手册调整 `gmac_mac_init` 内对 `DMA_RBSZ_MASK` 的写入。
- **`gmac_receive_frame(..., return_on_every_good_pkt=false, ...)`**：当前对 **`GMAC_MY_NORMAL_PKT_TYPE`** 在比对通过后仍返回 **false**（未实现参考代码中对 **`GMAC_MY_STOP_PKT_TYPE`** 的停止帧分支）；环回自测请保持 **`return_on_every_good_pkt == true`**，与 `test_mac_near_end_loopback_force_link` 一致即可。

---

## 4. 后续信息维护建议

在以下情况发生时，请在本 `README.md` 追加小节并标注 **日期**：

- SoC 固定 **CSR 基址、时钟、复位、AXI 位宽** 与 **SNPSVER** 实测值；
- 与 Databook 5.40a 某章节不一致的寄存器行为或 errata；
- 外接 PHY / RGMII 延时、RX/TX 时钟与 **LUD** 依赖关系；
- 启用 **TSN / ASP / 安全扩展** 后的初始化差异。

---

## 5. 文档与许可

- **Synopsys Databook** 为授权方保密资料，本 README 仅描述工程内**公开可得的 Linux 主线行为对齐**与**自研裸机接口**，不复制受控寄存器表全文。
- `baremetal_gmac_eqos` 内代码注释已标明与 Linux stmmac 的对齐关系；二次分发请遵守各自许可证（内核代码 GPL、本仓库裸机文件可按项目策略标注）。

---

*最后更新：整理自会话中关于 Linux 7.0.0 stmmac 路径、5.40a 适用性说明，以及 `baremetal_gmac_eqos` 实现与使用说明。*
