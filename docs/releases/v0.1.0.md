# C3M1 Firmware v0.1.0

首个适配 **C3M1 (C3MarkI) 量产硬件**、完成全链路实机验证的开源基础发行版。固件彻底脱离原厂双模式切换架构，采用**常驻 RAW 微单化取景**、**零快门时滞拍摄**与 **SDXC (exFAT / FAT32) 原生存储**。

- **发布日期**：2026-11-25
- **适用硬件**：C3M1 (C3MarkI) 量产主板（STM32F103ZET6 + OV7725 + AL422B + 2.0" 8-bit FSMC ST7789V3）
- **体积占用**：Flash `60,904 B`（~59.5 KB / 512 KB），SRAM `14,804 B`（~14.5 KB / 64 KB）

---

## 核心特性与改进

### 1. 常驻 RAW 微单化取景与零快门时滞
- **永久 RAW 管线**：OV7725 开机初始化一次后全程运行在 `644×480` 8-bit `GRBG` RAW（`COM3=0xD0` 硬件正向、`COM4=0x81` 6× PLL / 18 MHz PCLK / 45 fps、`COM2=0x01` 柔和驱动防 FPC 排线振铃），无模式切换、无软复位、无偶发错色行。
- **全档位 15 fps 流畅取景**：`1.5 KB` 零等待 SRAM LUT（扣 `BlackLevel=2` + 预览 Gray-World AWB + IEC 61966-2-1 sRGB OETF + RGB565 打包）配合 `DMA1_Channel1` MEM2MEM 双缓冲异步刷屏，`1X` 与慢门档全部稳跑 **15F**，`4X` 中心 `1:1`（`320×220`）对焦放大零延迟瞬切。
- **真零快门时滞（0~22 ms）**：快门 `1/8000 ~ 1/50` 与 `AUTO` 档预览曝光与拍照完全一致，按下快门直接锁存当前帧写卡；慢门 `1/25 ~ 2s` 预览保持 15F，按下快门后在 VSYNC 边界原子切入 `CLKRC=0x01` 单帧精确长曝（`1/8s ~ 2s` 居中显示 `EXPOSING...`）。
- **50Hz 抗频闪微单 P 曲线 AE**：`AUTO` 档覆盖 `1/23000s ~ 1/50s`（ISO `100 ~ 1600`），采用 3 段 50Hz 抗频闪 P 曲线、双速收敛追光（大误差快追 / 临近目标柔和刹车）、滞回高光保护与 VBLANK 原子写入，底栏实时显示 `SA:` / `IA:`。

### 2. 高可靠 DNG 1.4 出图与 `/CAYE_CAM/` 目录
- **根治慢卡断层撕裂**：拍照时先等 `g_vsync_state == 2` 硬件锁死 `FIFO_WEN = 0`，再执行 `f_open` / `f_write` / `stmflash_write`，彻底消除 AL422B 环形缓冲回绕断层。
- **硬件正向 DNG 1.4**：592 字节校准头（`Orientation = 1`、`CFAPattern = GRBG`、校准 `ColorMatrix1` 与 `NoiseProfile`），Adobe Camera Raw / Lightroom 直接正常打开，不再左右镜像。
- **目录归档**：挂载及拍照前自动创建 `/CAYE_CAM/` 目录，成片统一保存为 `/CAYE_CAM/CDI00001.dng` ~ `CDI65535.dng`。

### 3. 原生 SDXC（exFAT + FAT32，最高 2 TB）与安全热插拔
- **FatFs R0.15a**：开启 `FF_FS_EXFAT = 1` + `FF_USE_LFN = 1` + 32-bit LBA（支持至 2 TB SDXC 上限），原生兼容 FAT16 / FAT32 / exFAT。
- **STM32F1 HAL SD 驱动修复**：移除 3.3V 主控不适用的 `ACMD41 S18R`（1.8V 切换请求）以兼容严格 UHS-I / UHS-II SDXC 卡；防护 2 TB 边界 32-bit 溢出；修复热插拔 `disk.is_initialized` 缓存。
- **安全热插拔**：开启 `SDIO_CLOCK_POWER_SAVE`（空闲自动停 `SDIO_CK`），拔卡立即卸载并 `HAL_SD_DeInit()` 释放总线引脚，插卡 150 ms 消抖重挂并居中弹窗提示（`SD CARD READY` / `SD REMOVED` / `SD MOUNT FAIL!`）。
- **已实机验证卡型**：
  - Samsung EVO Plus 256 GB microSDXC（配 TF 转 SD 卡套，exFAT）
  - Lexar Professional 1800x 128 GB UHS-II SDXC（exFAT）
  - SDXC（exFAT）↔ SDHC（FAT32）运行中混插热插拔均正常

### 4. 顶板交互、胶片计数器与电源管理
- **16 档 BCD 快门拨盘**：`0..8` = `1/8000 ~ 1/50`，`9..14` = `1/25 ~ 2s`，`15` = `AUTO`。
- **胶片式过片计数**：底栏 `#00001` 在拉动过片扳手（`FILMKEY`）上弦进入 `RDY` 时自动推进到下一张待拍编号，拍完切 `WIND` 并居中无闪烁提示 `WIND FILM LEVER`（此时按快门静默忽略）。
- **内部 `VREFINT` 自校准电压**：通过 STM32 内部 1.20V 基准抵消板载 LDO 压降，底栏精确显示 `3.xxV`（`>=3.60V` 绿、`3.45~3.59V` 黄、`<3.45V` 红），低于 `3.25V` 时阻止写卡并提示 `LOW BATTERY!`（独立 ST-Link 供电 `<2.0V` 时不误触发）。
- **指示灯 `LED1 (PD9)`**：改为 5V 容忍开漏输出适配 `VCC5.0` 供电，实现取景全灭 / 曝光锁帧常亮 / 写卡 80 ms 闪烁 / 出错快闪 3 次。

---

## 发布文件与校验

| 文件 | 格式 | 起始地址 | 大小 | SHA-256 |
| :--- | :--- | :--- | :--- | :--- |
| `c3m1_camera_v0.1.0.hex` | Intel HEX（推荐） | 内含 `0x08000000` | 171,966 B | `12529ef0ea32c5ee2bb3afabf3580c2a93a724d14ba2bd65f4ea294f588d8fd3` |
| `c3m1_camera_v0.1.0.bin` | Raw Binary | `0x08000000` | 61,108 B | `2f4f7f9b0a65266996c6b688a951fbf5c2f1e1446ae10336878f920522901625` |

---

## 烧录说明

> **安全警告（必读）**
> 1. 主板 MicroUSB (`USB1`) 的 `VBUS` 直连 **`VCC3.3`**，**严禁直接插入电脑 5V USB 口**！请仅使用 ST-Link / CMSIS-DAP 3.3V SWD 接口连接（`Pin1=3.3V, Pin2=SWDIO, Pin3=SWCLK, Pin4=NRST, Pin5=GND`）。
> 2. **切勿使用 OTG MicroUSB 线**：`Pin4 (ID)` 直连 STM32 `NRST`，OTG 线内部将 `ID` 接地会导致芯片永久复位。请使用普通 4 芯非 OTG 线（悬空 `NRST`），并配合 `reset_config none`。

### OpenOCD + ST-Link（命令行）
```bash
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg \
  -c "reset_config none; program releases/v0.1.0/c3m1_camera_v0.1.0.hex verify reset exit"
```

### STM32CubeProgrammer / J-Flash
- 连接方式选择 **SWD**，Reset Mode 选择 **Software system reset**（因 4 芯线未接硬件 `NRST`）；
- 加载 `c3m1_camera_v0.1.0.hex`（或 `c3m1_camera_v0.1.0.bin` 指定起始地址 `0x08000000`），勾选 Verify 后点击 Download 即可。

---

## 已知硬件限制与后续计划
- **SD 卡侧面 `LOCK` 拨片无效**：C3M1 主板卡座 `CARD1` 的 `Pin 11 (WP)` 硬件悬空未接入 MCU，因此拨下 `LOCK` 滑块后本机仍可正常写入（属主板硬件设计）。
- **无 RTC 硬件**：机身电源开关硬切断电池且无 32.768 kHz 晶振，文件日期固定为 `2026-12-26`，DNG `DateTime` 标签固定为 `2026:08:17 00:00:00`。
- **`NUMRST (PA3)` 物理按键**：当前版本暂不做支持，后续统一纳入机内菜单 `RESET COUNT`。
- **TF 转 SD 卡套热插拔**：请连同外层大卡套一起插拔，避免仅拔内部小卡导致卡在位开关 `SDOK` 不触发。
