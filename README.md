# C3M1 (C3MarkI) Camera Firmware

适配 **C3M1 / C3MarkI 量产硬件** 的开源 STM32F103ZET6 RAW 数码相机固件（脱离 STM32CubeIDE，支持 macOS / Linux 命令行 `make` 构建）。

## 已适配功能

- **实时取景**：OV7725 QVGA (`320×240` RGB565) → AL422B FIFO (`PG1~PG8`) → 8 位 8080 FSMC ST7789V3 (`0x600003FF`/`0x60000400`)，VSYNC 三态机防撕裂。
- **DNG RAW 拍照**：快门触发后切换至 VGA `644×480` Bayer RAW，封装 592 字节校准版 DNG 1.4 文件头（含 180° 方向标记与 `ColorMatrix1`，有效裁切 `640×480`），4KB 块缓冲写入 SD 卡生成 `0:CDI00000.dng` ~ `0:CDI65535.dng`。
- **顶板完整交互**：
  - `KEYSHOT (PF2)`：快门拍照（写卡期间 `LED1 (PD9)` 点亮）。
  - `TSR1/2/4/8 (PF6~PF9)`：16 档 BCD 快门速度旋钮（`0=AUTO`，`1..15` 手动 `1/1000s ~ 1/8s`）。
  - `KEYISO (PF1)`：循环切换 ISO（`AUTO / 100 / 200 / 400 / 800 / 1600`）。
  - `KEY4X (PF3)`：切换 VGA 原生中心 `160×120` 2×2 放大对焦辅助（4X 面积放大）。
  - `FILMKEY (PF10)`：过片扳手上弦（拍完一张需拨动过片扳手进入 `RDY`；可在 `Core/Src/main.c` 调整 `C3M1_REQUIRE_FILM_LEVER` 开关）。
  - `NUMRST (PA3)`：开机按住或运行中长按 1.5s 将内部 Flash (`0x08070000`) 快门计数归零。
- **外设与 OSD 状态栏**：
  - `SDOK (PB7)`：SD 卡在位检测与热插拔自动挂载/卸载（无卡安全启动，不会卡死）。
  - `ADC1_IN1 (PA1)`：10k/10k 锂电池分压采样与电量百分比显示。
  - 底部 20px 无闪烁 OSD 状态栏：实时显示快门、ISO、`4X`、计数、状态（`RDY`/`WIND`/`SAVE`/`NOSD`）与电量。

## 硬件引脚对照（C3MarkI 量产板）

| 模块 | 引脚分配 | 说明 |
| :--- | :--- | :--- |
| **8 位 FSMC LCD** | `D0~D3`: `PD14/15/0/1`, `D4~D7`: `PE7~10`, `RD`: `PD4`, `WR`: `PD5`, `DC/RS`: `PG0(A10)` | `CS` 顶板接地，Bank1 NE1 (`0x60000000`) |
| **LCD 控制** | `LCDEN(BL)`: `PF4` (高有效), `LCDRST`: `PF5` (低复位) | ZJY200K-IF09 2.0" IPS ST7789V3 |
| **OV7725 + AL422B** | `D0~D7`: `PG1~PG8`, `VSYNC`: `PA8`, `SCL`: `PD3`, `SDA`: `PG13`, `WRST`: `PD6`, `RRST`: `PG14`, `OE`: `PG15`, `WEN`: `PB3`, `RCLK`: `PB4`, `U7 OE#`: `PD11(低使能)`, `/WE`: `PD10(高阻输入)` | AL422B `/RE` 硬件接地 |
| **顶板按键与机构** | `KEYISO`: `PF1`, `KEYSHOT`: `PF2`, `KEY4X`: `PF3`, `FILMKEY`: `PF10` (外置上拉，低有效)；`TSR1/2/4/8`: `PF6~PF9` (内部下拉，高有效 BCD) | 详见 `Drivers/BSP/KEY/` |
| **SDIO + 检测** | `D0~D3`: `PC8~11`, `CK`: `PC12`, `CMD`: `PD2`, `SDOK`: `PB7` (0=有卡) | 1-bit 初始化后切 4-bit |
| **其他** | `BAT_ADC`: `PA1(ADC1_IN1)`, `NUMRST`: `PA3`, `LED1`: `PD9` (低亮), `USART1`: `PA9/PA10` (115200) | 串口输出调试日志 |

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
> **安全警告**：主板 MicroUSB (`USB1`) 的 `VBUS` 直连 **`VCC3.3`**，**严禁直接插普通 USB 线连电脑 5V**！必须使用外部 CMSIS-DAP / ST-Link 通过定制 SWD 转接线烧录（`Pin1=3.3V, Pin2=SWDIO, Pin3=SWCLK, Pin4=NRST, Pin5=GND`）。

```bash
make flash          # CMSIS-DAP / DAPLink (dap.cfg)
make flash-stlink   # ST-Link
```

如需恢复出厂固件，直接烧录原资料包中的 `软件部分/C3M1生产烧录.hex`。
