# C3M1 (C3MarkI) Camera Firmware

适配 **C3M1 / C3MarkI 量产硬件** 的开源 STM32F103ZET6 RAW 数码相机固件（脱离 STM32CubeIDE，支持 macOS / Linux 命令行 `make` 构建与 ST-Link / CMSIS-DAP 烧录）。

> 详细架构设计、硬件排坑记录与时序原理见 [docs/architecture.md](./docs/architecture.md)；原厂二进制固件的最小化定向补丁见 [c3m1-factory-patch/](./c3m1-factory-patch/README.md)。

## 核心特性

- **常驻 RAW 微单化取景与零快门时滞**：
  - OV7725 开机初始化一次后全程常驻 `644×480` `GRBG` 片内 10-bit Gamma 压扩 RAW（`COM7=0x01`、`DSP_Ctrl1=0x87`、`DSP_Ctrl4=0x02`、`COM13=0xE6` 硬件 ABLC 防热机高 ISO 温漂、`COM3=0xD0` 硬件正向、`COM4=0x81` 6× PLL + `CLKRC=0x00` 18 MHz PCLK / 45 fps / `20.9 ms` 卷帘扫描、`COM2=0x01` 柔和默认驱动防排线振铃），彻底消除取景↔拍照模式切换、软复位与 5 帧过渡等待。
  - 取景器采用 `1.5 KB` 零等待 SRAM 查表（经 512 B `s_inv_gamma_10` 还原 10-bit 线性光 + 按当前增益动态扣 10-bit `BlackLevel = 5 + gain_x` + 预览 Gray-World AWB + 10-bit 分数插值 IEC sRGB OETF + RGB565 打包）与 `DMA1` 双缓冲 MEM2MEM FSMC 异步刷屏流水线：`1X` 模式 `320×110` 逐行双倍输出（保持水平 320 清晰度，~38 ms 刷屏 / **稳跑 15 fps**，暗部监看更平滑），`4X` 模式直接裁切中心 `320×220` 做 `1:1` 原生像素对焦放大（零延迟瞬切）。
  - **快门 `1/8000 ~ 1/25` 与 `AUTO`**：预览即真实拍照曝光，按下快门 `0 ~ 22 ms` 真零时滞直接锁存并写出当前帧。
  - **慢门 `1/8s ~ 2s`**：取景传感器保持 45 fps（`500` 行、`DM_LN=0`）并由硬件 `GAIN`（最高 `31×`）+ 数字 LUT（最高 `4×`）补偿预览亮度，实现**慢门档同样稳跑 15 fps**；按下快门时在 VSYNC 边界原子切入 `CLKRC=0x01` 与真实低 ISO 长曝寄存器，单帧精确曝光并写卡后自动切回 45 fps 预览。
- **高可靠 DNG 1.4 拍照与存储**：
  - 先完成 `g_vsync_state == 2` 锁死 `FIFO_WEN = 0` 再执行 `f_open` / `f_write` / `stmflash_write`，彻底根除慢速 SD 卡下 AL422B 384 KB 环形回绕造成的画面断层撕裂。
  - **更小体积、更高暗部精度（310 KB vs 原厂 618 KB）**：原厂固件将线性 8-bit RAW 每个像素补 1 字节 `0x00` 硬塞进 16-bit 容器，文件虽达 `618 KB`，但实际仍只有线性 8-bit 精度（传感器 10-bit ADC 低 2 bit 被直接丢弃）；本固件改为 **OV7725 片内 10-bit Gamma 压扩 + 硬件 ABLC 抗温漂（`COM13=0xE6`）+ 紧凑 8-bit 封装 + DNG 标准 `LinearizationTable`（Tag `0xC618`）自动线性还原**（`1,120 B` 文件头，按拍照 ISO 动态写入精确 `BlackLevel = 6..21`、`WhiteLevel = 1016`、`Orientation = 1`、`CFAPattern = GRBG`、校准 `ColorMatrix1` 与 `NoiseProfile`），**文件体积减半（仅 310 KB）、写卡提速 2×，同时实际暗部灰阶精细度反而是原厂的 2.6 倍**（Adobe Camera Raw / Lightroom 自动线性还原），统一自动建目录保存至 `/CAYE_CAM/CDI00001.dng` ~ `/CAYE_CAM/CDI65535.dng`。
  - **原生 SDXC / exFAT 存储支持（最高 2 TB）**：升级至 **FatFs R0.15a**（开启 `FF_FS_EXFAT` + `FF_USE_LFN`，原生兼容 FAT16/FAT32/exFAT），并修复 STM32F1 HAL SD 驱动（移除 3.3V 主控不适用的 `S18R` 1.8V 切换请求、防护 2 TB 边界 32-bit 溢出），64 GB ~ 2 TB 出厂 exFAT SDXC 卡即插即用（已实机验证：三星 EVO Plus 256 GB TF 转 SD、雷克沙 Professional 1800x 128 GB UHS-II SDXC，且支持 SDXC/SDHC 混插热插拔）。
- **顶板完整交互与统一 UI 反馈**：
  - `TSR1/2/4/8 (PF6~PF9)`：16 档 BCD 快门拨盘全映射：
    - `0..8`：机顶物理刻度 `1/8000, 1/4000, 1/2000, 1/1000, 1/500, 1/300, 1/150, 1/100, 1/50`
    - `9..14`：扩展慢门 `1/25, 1/8, 1/4, 1/2, 1s, 2s`
    - `15`：`AUTO` 自动测光档（4 段微单 P 曲线软件闭环 AE，底栏实时显示 `SA:1/xxx` 与自动 ISO `IA:xxx`）
  - `KEYISO (PF1)`：循环切换 ISO（`AUTO / 100 / 200 / 400 / 800 / 1600`，`AUTO` 时底栏以 `IA:` 显示 1/3 档实时感光度）。
  - `KEY4X (PF3)`：瞬时切换 `1X` 全视野 / `4X` 中心 `1:1` 放大对焦。
  - `FILMKEY (PF10)`：过片扳手上弦（未上弦时居中无闪烁显示 `WIND FILM LEVER`，此时按快门完全静默忽略不卡顿；可在 `Core/Src/main.c` 调整 `C3M1_REQUIRE_FILM_LEVER`）。
  - `KEYSHOT (PF2)`：快门拍照（慢门档居中显示 `EXPOSING...`）。
  - `NUMRST (PA3)`：开机按住或运行中长按 1.5s 将内部 Flash (`0x08070000`) 快门计数归零。
  - `LED1 (PD9)`：开漏输出适配 `VCC5.0` 供电，实现 **取景全灭 / 曝光与锁帧常亮 / 写卡 80 ms 闪烁 / 写卡异常快闪 3 次**。
  - 底部 20px 状态栏：显示快门（`S:/SA:`）、ISO（`I:/IA:`）、`4X` 或实时帧率（`15F`）、胶片式张数 `#00001`（过片上弦后即切到下一张编号）、状态（`RDY`/`WIND`/`EXP`/`SAVE`/`NOSD`）与内部 `VREFINT` 自校准电池电压（`3.xxV`，`<3.60V` 黄、`<3.45V` 红、`<3.25V` 低电写卡保护）。
  - SD 卡安全热插拔：开启 `SDIO_CLOCK_POWER_SAVE`（空闲自动停 `SDIO_CK`），拔卡立即卸载并 `DeInit` 释放引脚，插卡 150ms 消抖重挂并居中弹窗提示（注：使用 TF 转 SD 卡套时请务必连同外层卡套整体插拔；另因主板卡座 `CARD1` 的 `Pin 11 (WP)` 硬件悬空未接入 MCU，全尺寸 SD 卡侧面的物理 `LOCK` 写保护拨片在本机上不生效，拨下后仍可正常写入）。

## 硬件引脚对照（C3MarkI 量产板）

| 模块 | 引脚分配 | 说明 |
| :--- | :--- | :--- |
| **8 位 FSMC LCD** | `D0~D3`: `PD14/15/0/1`, `D4~D7`: `PE7~10`, `RD`: `PD4`, `WR`: `PD5`, `DC/RS`: `PG0(A10)` | `CS` 顶板接地，Bank1 NE1 (`0x60000000`)，配合 `DMA1_Channel1` 异步直写 |
| **LCD 控制** | `LCDEN(BL)`: `PF4` (高有效), `LCDRST`: `PF5` (低复位) | ZJY200K-IF09 2.0" IPS ST7789V3 |
| **OV7725 + AL422B** | `D0~D7`: `PG1~PG8`, `VSYNC`: `PA8`, `SCL`: `PD3`, `SDA`: `PG13`, `WRST`: `PD6`, `RRST`: `PG14`, `OE`: `PG15`, `WEN`: `PB3`, `RCLK`: `PB4`, `U7 OE#`: `PD11(低使能)`, `/WE`: `PD10(高阻输入)` | AL422B `/RE` 硬件接地，`U4` 实现 `OV_WEN NAND HREF -> /WE` |
| **顶板按键与机构** | `KEYISO`: `PF1`, `KEYSHOT`: `PF2`, `KEY4X`: `PF3`, `FILMKEY`: `PF10` (外置上拉，低有效)；`TSR1/2/4/8`: `PF6~PF9` (内部下拉，高有效 BCD) | 详见 `Drivers/BSP/KEY/` |
| **SDIO + 检测** | `D0~D3`: `PC8~11`, `CK`: `PC12`, `CMD`: `PD2`, `SDOK`: `PB7` (0=有卡) | 1-bit 初始化后切 4-bit，支持热插拔自动挂载/卸载；卡座 `Pin 11 (WP)` 硬件悬空（`LOCK` 写保护拨片无效） |
| **其他** | `BAT_ADC`: `PA1(ADC1_IN1)`, `NUMRST`: `PA3`, `LED1`: `PD9` (**5V 容忍开漏低亮**), `USART1`: `PA9/PA10` (115200) | 串口输出帧率/测光/写卡日志 |
| **Type-C 充电** | `USB3 (TYPE-C 6P)` → `TP4056` | **硬件避坑**：主板将 `CC1 (A5)` 与 `CC2 (B5)` 短接共用单颗 `5.1k (R20)`，插带 e-Marker 的 C-to-C 快充线会被 PD 头误判为音频配件拒绝供电，请使用 **USB-A 转 USB-C 线** 或无 e-Marker 的普通 3A C-to-C 线充电 |

## 实机联调微调项

若不同批次屏幕出现颜色反转或方向偏差，只需调整 `Drivers/BSP/LCD/lcd.h` 中两个宏（无需改逻辑）：
- `LCD_ST7789_INVERSION`：`1` 开启反相 (`0x21`，IPS 屏常用) / `0` 关闭反相 (`0x20`)
- `LCD_ST7789_RGB_ORDER`：`0x00` (RGB) / `0x08` (BGR)
- 若需上下 180° 翻转取景画面，可在 `Drivers/BSP/LCD/lcd.c` 的 `lcd_display_dir(1)` 中调整默认扫描方向。

## 构建与烧录（macOS / Linux）

### 1. 安装工具链
- **macOS**：
  ```bash
  brew install --cask gcc-arm-embedded
  brew install openocd
  ```
- **Ubuntu / Debian**：
  ```bash
  sudo apt install gcc-arm-none-eabi make openocd
  ```

### 2. 编译
```bash
make -j
```
产物输出至 `build/`：`c3m1_camera.elf`、`c3m1_camera.hex`、`c3m1_camera.bin`、`c3m1_camera.map`。

### 3. SWD 烧录（MicroUSB 复用口）
> **安全警告**：
> 1. 主板 MicroUSB (`USB1`) 的 `VBUS` 直连 **`VCC3.3`**，**严禁直接插电脑 5V USB 口**！必须通过外部 ST-Link / CMSIS-DAP 供电与连接（`Pin1=3.3V, Pin2=SWDIO, Pin3=SWCLK, Pin4=NRST, Pin5=GND`）。
> 2. **切勿使用 OTG MicroUSB 线**：MicroUSB `Pin4 (ID)` 在主板上直连 STM32 `NRST`，OTG 线内部将 `ID` 与 `GND` 短接会导致芯片被永久拉低复位而无法烧录。请使用普通 4 芯非 OTG MicroUSB 线（悬空 `NRST`），并配合 `reset_config none`。

```bash
make flash-stlink   # ST-Link (已内置 reset_config none)
make flash          # CMSIS-DAP / DAPLink (dap.cfg)
```

如需恢复出厂固件，直接烧录原资料包中的 `软件部分/C3M1生产烧录.hex` 或使用 `c3m1-factory-patch/` 中的定向修复版。

## 许可证

本作品遵循以下法律许可：[Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International (CC BY-NC-SA 4.0)](https://creativecommons.org/licenses/by-nc-sa/4.0/)，详见 [LICENSE](./LICENSE)。

- 第三方底层库（STM32F1 HAL / CMSIS、FatFs）继续遵循其各自原有许可协议。
- 原始硬件与基线代码来源：C3M1 (C3MarkI) 开源资料包（基于正点原子 STM32F103 示例改造），本仓库在其基础上完成量产板适配、常驻 RAW 微单化重构与缺陷修复，按相同许可共享，仅供非商业学习研究使用。
