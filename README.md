# C3M1 Camera Firmware

基于 `C3M1开源资料包（更新视频）/软件部分/C3M1_Camera` 提取整理的独立固件代码仓库（已移除 IDE 产物和 Windows 绝对路径，附跨平台 `Makefile`）。

> **重要提示**：初始提交（baseline）为原开源工程代码（正点原子开发板 16 位 LCD + `PC0~PC7` 摄像头数据口版本），**尚未适配 C3M1 量产板**（8 位 LCD + `PG1~PG8` + 顶板按键/拨盘/过片扳手）。请先完成硬件适配再烧录至量产整机；如需恢复出厂固件，请使用资料包中的 `软件部分/C3M1生产烧录.hex`。

## 目录结构

- `Core/`：应用主程序（`main.c`）、外设初始化（`gpio.c`/`fsmc.c`/`sdio.c`/`tim.c`/`usart.c`）、中断与启动汇编
- `Drivers/BSP/`：板级驱动（`OV7725`、`EXTI`、`LCD`、`KEY`、`LED`、`STMFLASH`、`TPAD`、`BEEP`）
- `Drivers/SYSTEM/`：系统时钟与延时工具（`delay`、`sys`）
- `Drivers/STM32F1xx_HAL_Driver/` & `Drivers/CMSIS/`：STM32F1 HAL 库与 CMSIS
- `FATFS/` & `Middlewares/Third_Party/FatFs/`：FatFs R0.11 文件系统与 SDIO 驱动适配
- `STM32F103ZETX_FLASH.ld`：STM32F103ZET6 链接脚本（512KB Flash / 64KB RAM）
- `32_OV7725_Camera.ioc`：STM32CubeMX 配置文件
- `dap.cfg`：OpenOCD CMSIS-DAP SWD 烧录配置
- `Makefile`：macOS / Linux 跨平台构建脚本

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
产物输出在 `build/` 目录：`c3m1_camera.elf`、`c3m1_camera.hex`、`c3m1_camera.bin`、`c3m1_camera.map`。

### 3. 烧录（SWD）
```bash
# 使用 CMSIS-DAP / DAPLink
make flash

# 使用 ST-Link
make flash-stlink
```
