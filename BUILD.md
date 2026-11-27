# 构建与烧录指南（macOS / Linux）

本文档介绍如何在 macOS / Linux 下脱离 STM32CubeIDE，使用命令行工具链编译与烧录 `c3m1-firmware`。

---

## 1. 安装工具链

- **macOS**：
  ```bash
  brew install --cask gcc-arm-embedded
  brew install openocd
  ```
- **Ubuntu / Debian**：
  ```bash
  sudo apt update
  sudo apt install gcc-arm-none-eabi libnewlib-arm-none-eabi make openocd
  ```

---

## 2. 编译固件

```bash
make -j
```

构建产物输出至 `build/` 目录：
- `build/c3m1_camera.elf`（带调试符号）
- `build/c3m1_camera.hex`（ST-Link 烧录推荐）
- `build/c3m1_camera.bin`（裸二进制）
- `build/c3m1_camera.map`（内存与符号映射表）
- `build/fw_version.h`（自动生成）

### 版本号与短哈希自动注入
`Makefile` 在每次编译前会自动生成 `build/fw_version.h`（带 `cmp -s` 变更检测，未变化时不触发多余重编）：
- `FW_VERSION_STR` / `FW_VER_MAJOR` / `FW_VER_MINOR` / `FW_VER_PATCH`：由 `Makefile` 中的 `FW_VERSION`（当前 `v0.1.15`，可通过 `make FW_VERSION=vX.Y.Z` 覆盖）解析，既用于开机画面与菜单底栏显示，也写入 SD 卡二进制配置 `0:CAYE_CFG/c3m1.cfg` 做固件版本校验。
- `FW_GIT_HASH_STR`：来自 `git rev-parse --short HEAD`；当工作区存在未提交改动时自动追加 `-dirty` 后缀（例如 `cf8b90d-dirty`），便于区分正式构建与本地调试版本。

如需清理全部构建产物：
```bash
make clean
```

---

## 3. 实机屏幕批次微调（可选）

若不同批次 2.0" ST7789V3 屏幕出现颜色反转或方向偏差，只需修改 `Drivers/BSP/LCD/lcd.h` 中对应宏后重新 `make -j`（无需改动业务逻辑）：
- `LCD_ST7789_INVERSION`：`1` 开启反相 (`0x21`，IPS 屏常用) / `0` 关闭反相 (`0x20`)
- `LCD_ST7789_RGB_ORDER`：`0x00` (RGB) / `0x08` (BGR)
- 如需上下 180° 翻转取景画面，可在 `Drivers/BSP/LCD/lcd.c` 的 `lcd_display_dir(1)` 中调整默认扫描方向。

---

## 4. SWD 烧录（MicroUSB 复用口）

> **安全警告**：
> 1. 主板 MicroUSB (`USB1`) 的 `VBUS` 直连 **`VCC3.3`**，**严禁直接插电脑 5V USB 口**！必须通过外部 ST-Link / CMSIS-DAP 提供 3.3V 供电与 SWD 连接（引脚定义：`Pin1=3.3V, Pin2=SWDIO, Pin3=SWCLK, Pin4=NRST, Pin5=GND`）。
> 2. **切勿使用 OTG MicroUSB 线**：MicroUSB `Pin4 (ID)` 在主板上直连 STM32 `NRST`，OTG 线内部将 `ID` 与 `GND` 短接会导致芯片被永久拉低复位而无法连接烧录（OpenOCD 报 `Halt timed out` / `DHCSR=0x02010001`）。请使用普通 **4 芯非 OTG MicroUSB 线**（悬空 `NRST`），配合 `reset_config none` 即可稳定烧录。

### 使用 ST-Link V2（推荐）
```bash
make flash-stlink
```
（已内置 `interface/stlink.cfg`、`target/stm32f1x.cfg` 与 `reset_config none`，自动校验并复位启动。）

### 使用 CMSIS-DAP / DAPLink
```bash
make flash
```
（使用仓库根目录的 `dap.cfg`。）

### 恢复出厂固件
如需恢复出厂固件，直接烧录原资料包中的 `软件部分/C3M1生产烧录.hex`，或使用 [c3m1-factory-patch/](./c3m1-factory-patch/README.md) 中已验证的定向补丁版。

---

## 5. 串口调试日志（可选）
- 引脚：`USART1`（`PA9=TX`、`PA10=RX`），波特率 `115200 8N1`。
- 开机打印固件版本号与哈希、电池电压、SD 卡挂载与文件系统类型、`CAYE_CFG/c3m1.cfg` 读写校验结果，以及实时帧率、测光与 DNG 写卡耗时。
