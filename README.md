# C3M1 (C3MarkI) Camera Firmware

适配 **C3M1 / C3MarkI 量产硬件** 的开源 STM32F103ZET6 RAW 数码相机固件。

> - 编译、烧录与屏幕微调指南见 **[BUILD.md](./BUILD.md)**
> - 架构设计、硬件排坑与 OV7725 实测见 **[docs/architecture.md](./docs/architecture.md)** 与 **[docs/ov7725_experiments.md](./docs/ov7725_experiments.md)**
> - 版本变更记录见 **[CHANGELOG.md](./CHANGELOG.md)**；原厂二进制固件定向补丁见 **[c3m1-factory-patch/](./c3m1-factory-patch/README.md)**

---

## 核心特性

- **常驻 RAW 微单化取景与零快门时滞**：
  - OV7725 开机初始化一次后全程常驻 `644×480` `GRBG` 片内 10-bit Gamma 压扩 RAW（`COM7=0x01`、`DSP_Ctrl1=0x87`、`DSP_Ctrl4=0x02`、`COM13=0xE6` 硬件 ABLC 防热机高 ISO 温漂、`COM3=0xD0` 硬件正向、`COM4=0x81` 6× PLL / 18 MHz PCLK / 45 fps / `20.9 ms` 卷帘扫描、`COM2=0x01` 柔和驱动防排线振铃），彻底消除取景↔拍照模式切换与软复位等待。
  - 取景器采用 `1.5 KB` 零等待 SRAM 查表（经 `s_inv_gamma_10` 还原 10-bit 线性光 + 动态扣黑电平 + 预览 Gray-World AWB + 10-bit 分数插值 IEC sRGB OETF + RGB565 打包）与 `DMA1` 双缓冲异步刷屏：`1X` 模式稳跑 **15 fps**，`4X` 模式直接裁切中心 `320×220` 做 `1:1` 原生像素对焦放大（零延迟瞬切）。
  - **快门 `1/8000 ~ 1/25` 与 `AUTO`**：预览即真实拍照曝光，按下快门 `0 ~ 22 ms` 真零时滞直接锁存并写出当前帧。
  - **慢门 `1/8s ~ 2s`**：取景保持 45 fps（由硬件 `GAIN` + 数字 LUT 补偿预览亮度，**全档位稳跑 15 fps**）；按下快门时在 VSYNC 边界原子切入真实低 ISO 慢门曝光，单帧锁存后自动切回高速预览。
- **高可靠 310 KB 10-bit Gamma 压扩 DNG 1.4**：
  - 严格先锁死 AL422B `FIFO_WEN = 0` 再执行文件 I/O 与 Flash 写入，彻底根除慢卡环形回绕断层撕裂。
  - 相比原厂将线性 8-bit 零填充进 16-bit 容器的 `618 KB` 文件，本固件采用 **片内 10-bit Gamma 压扩 + 硬件 ABLC（`COM13=0xE6`）+ 紧凑 8-bit 封装 + DNG 12-bit 精度 `LinearizationTable`（Tag `0xC618`, `0..4064`）与分通道动态 `GRBG BlackLevel`**，单张仅 **310 KB**（体积减半、写卡提速 2×），实际暗部灰阶精细度反而达原厂 **2.62×**（Adobe Camera Raw / Lightroom 自动线性还原），统一保存至 `/CAYE_CAM/CDI00001.dng` ~ `/CAYE_CAM/CDI65535.dng`。
- **原生 SDXC / exFAT 存储支持（最高 2 TB）**：
  - 基于 **FatFs R0.15a**（开启 `FF_FS_EXFAT` + `FF_USE_LFN` + `FF_USE_MKFS`），兼容 FAT16 / FAT32 / exFAT，并修复 STM32F1 HAL SD 驱动（去除 3.3V 主控不适用的 `S18R`、防护 2 TB 边界 32-bit 溢出），支持 64 GB ~ 2 TB 出厂 exFAT SDXC 卡即插即用与 SDXC/SDHC 混插热插拔（已实机验证三星 EVO Plus 256 GB、雷克沙 1800x 128 GB UHS-II）。

---

## 操控说明

### 顶板物理控件
- **快门波轮 `TSR1/2/4/8 (PF6~PF9)`（16 档 BCD）**：
  - `0..8`：机顶刻度 `1/8000, 1/4000, 1/2000, 1/1000, 1/500, 1/300, 1/150, 1/100, 1/50`
  - `9..14`：扩展慢门 `1/25, 1/8, 1/4, 1/2, 1s, 2s`
  - `15`：`AUTO` 自动测光档（4 段可配置微单 P 曲线软件闭环 AE，底栏实时显示 `SA:1/xxx` 与 `IA:xxx`）
- **`KEYISO / ASA (PF1)`**：
  - **短按**（松手触发）：循环切换 ISO（`AUTO / 100 / 200 / 400 / 800 / 1600`），并自动防抖保存至 SD 卡，下次开机恢复上次 ISO 模式。
  - **长按 ≥ 600 ms**：进入纯英文菜单系统（未插 SD 卡时拒绝进入）。
- **`KEY4X (PF3)`**：瞬时切换 `1X` 全视野 / `4X` 中心 `1:1` 放大对焦。
- **`FILMKEY (PF10)`**：过片扳手上弦（未上弦时居中显示 `WIND FILM LEVER` 并静默忽略快门；可在菜单 `FILM LEVER` 中实时关闭）。
- **`KEYSHOT (PF2)`**：快门拍照（慢门档居中显示 `EXPOSING...`）。

### 状态栏与指示灯
- **底部 20px 状态栏**：显示快门（`S:/SA:`）、ISO（`I:/IA:`）、`4X` 或实时帧率（`15F`）、胶片式张数 `#00001`（上弦后即切至下一张编号）、状态（`RDY`/`WIND`/`EXP`/`SAVE`/`NOSD`）与内部 `VREFINT` 自校准电池电量（菜单可选 `3.xxV` 电压或 `xx%` 百分比，`<3.60V` 黄、`<3.45V` 红、`<3.25V` 低电写卡保护）。
- **`LED1 (PD9)`**：开漏输出适配 `VCC5.0` 供电 — **取景全灭 / 曝光与锁帧常亮 / 写卡 80 ms 闪烁 / 写卡异常快闪 3 次**。

---

## 菜单系统与配置（Menu & `CAYE_CFG`）

### 进入与交互
- **进入条件**：必须已插入并正常挂载 SD 卡；在取景界面**长按 `KEYISO (ASA)` ≥ 600 ms** 即可进入纯英文菜单。未插卡时弹窗提示 `NO SD CARD!` 并拒绝进入；若菜单打开期间热拔 SD 卡，将立即安全退出并卸载文件系统。
- **操作映射**：
  - **旋转快门波轮**：上下选择菜单项（支持跨 `15↔0` 循环与滚动条翻页），在确认弹窗中切换 `NO / YES`。
  - **按 `KEYSHOT`（快门）**：循环切换当前设置项（即时保存至 SD 卡）或确认执行子操作。
  - **短按 `KEYISO (ASA)`**：返回上一级或退出菜单（退出时自动等待按键松开防误拍，并同步波轮当前物理快门刻度）。
- **版本与哈希底栏**：菜单最底部常驻显示当前固件版本号与 Git 短哈希（工作区未提交修改时自动带 `-dirty`，例如 `v0.1.2 (22a7691)`）以及配置状态（`CFG OK` / `SAVED` / `SD ERR`）。

### 菜单项一览
| 菜单项 | 可选值 / 操作 | 说明 |
| :--- | :--- | :--- |
| **`MAX AUTO ISO`** | `400` / `800` / `1600`（默认 `1600`） | `AUTO` 自动 ISO 上限（手动 ISO 仍可正常选至 `1600`） |
| **`MIN AUTO SHT`** | `1/100` / `1/50` / `1/25` / `1/15` / `1/8`（默认 `1/50`） | `AUTO` 快门最慢下限；慢于 `1/50` 时取景仍保持 15F，拍照自动走单帧原子慢门 |
| **`ANTI-FLICKER`** | `50HZ` / `60HZ` / `OFF`（默认 `50HZ`） | 室内工频抗频闪折点（`50HZ: 1/100↔1/50`，`60HZ: 1/120↔1/60`，`OFF` 无阶梯） |
| **`BATTERY DISP`** | `VOLT` / `PCT`（默认 `VOLT`） | 底栏电量显示格式：`3.xxV` 实时电压 或 10 段锂电百分比 `xx%` |
| **`FILM LEVER`** | `ON` / `OFF`（默认 `ON`） | 是否要求每次拍照前拨动过片扳手上弦 |
| **`FORMAT SD`** | `RUN >`（`NO / YES` 二次确认） | 机内格式化 SD 卡（自动按容量选 FAT32/exFAT，`<3.25V` 低电禁格，完成后自动重建 `/CAYE_CAM` 与配置文件） |
| **`SYSTEM INFO`** | `VIEW >` | 查看文件系统类型、SD 总容量/剩余容量、当前快门计数、电池电压与百分比、配置路径及固件版本哈希 |
| **`EXIT MENU`** | `BACK` | 退出菜单返回实时取景 |

### 二进制配置文件（`0:CAYE_CFG/c3m1.cfg`）
- 所有菜单项与**关机前最后一次 ISO 模式**统一以定长 **32 字节二进制格式**固化在 SD 卡 `CAYE_CFG/c3m1.cfg` 中（不消耗 STM32 内部 Flash 擦写寿命）。
- 每次开机或热插卡挂载时依次执行 6 重校验：**文件大小 == 32 B → `C3M1` 魔数与结构版本 → 固件版本 `major.minor.patch` 匹配 → IEEE 802.3 CRC-32 → 字段枚举范围**；任意校验不通过（如跨版本升级固件或文件损坏）时自动安全回退默认值并重写有效配置。

---

## 硬件引脚对照与使用注意（C3MarkI 量产板）

| 模块 | 引脚分配 | 说明 |
| :--- | :--- | :--- |
| **8 位 FSMC LCD** | `D0~D3`: `PD14/15/0/1`, `D4~D7`: `PE7~10`, `RD`: `PD4`, `WR`: `PD5`, `DC/RS`: `PG0(A10)` | `CS` 顶板接地，Bank1 NE1 (`0x60000000`)，配合 `DMA1_Channel1` 异步直写 |
| **LCD 控制** | `LCDEN(BL)`: `PF4` (高有效), `LCDRST`: `PF5` (低复位) | ZJY200K-IF09 2.0" IPS ST7789V3 |
| **OV7725 + AL422B** | `D0~D7`: `PG1~PG8`, `VSYNC`: `PA8`, `SCL`: `PD3`, `SDA`: `PG13`, `WRST`: `PD6`, `RRST`: `PG14`, `OE`: `PG15`, `WEN`: `PB3`, `RCLK`: `PB4`, `U7 OE#`: `PD11`, `/WE`: `PD10(高阻)` | AL422B `/RE` 接地，`U4` 实现 `OV_WEN NAND HREF -> /WE` |
| **顶板按键与机构** | `KEYISO`: `PF1`, `KEYSHOT`: `PF2`, `KEY4X`: `PF3`, `FILMKEY`: `PF10` (外置上拉低有效)；`TSR1/2/4/8`: `PF6~PF9` (内部下拉高有效 BCD) | 详见 `Drivers/BSP/KEY/` |
| **SDIO + 检测** | `D0~D3`: `PC8~11`, `CK`: `PC12`, `CMD`: `PD2`, `SDOK`: `PB7` (0=有卡) | 支持规范热插拔（TF 转 SD 请连同卡套整体插拔）；主板卡座 `Pin 11 (WP)` 悬空，SD 卡物理 `LOCK` 写保护拨片不生效 |
| **其他** | `BAT_ADC`: `PA1(ADC1_IN1)`, `LED1`: `PD9` (**5V 容忍开漏低亮**), `USART1`: `PA9/PA10` (115200) | 串口输出版本、测光、配置与写卡日志 |
| **Type-C 充电** | `USB3 (TYPE-C 6P)` → `TP4056` | **硬件避坑**：主板将 `CC1 (A5)` 与 `CC2 (B5)` 短接共用单颗 `5.1k (R20)`，插带 e-Marker 的 C-to-C 快充线会被 PD 头误判为音频配件拒绝供电，请使用 **USB-A 转 USB-C 线** 或无 e-Marker 的普通 3A C-to-C 线充电 |

---

## 许可证

本作品遵循以下法律许可：[Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International (CC BY-NC-SA 4.0)](https://creativecommons.org/licenses/by-nc-sa/4.0/)，详见 [LICENSE](./LICENSE)。

- 第三方底层库（STM32F1 HAL / CMSIS、FatFs）继续遵循其各自原有许可协议。
- 原始硬件与基线代码来源：C3M1 (C3MarkI) 开源资料包（基于正点原子 STM32F103 示例改造），本仓库在其基础上完成量产板适配、常驻 RAW 微单化重构、菜单配置系统与缺陷修复，按相同许可共享，仅供非商业学习研究使用。
