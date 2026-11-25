# C3M1 原厂固件二进制补丁（已实机验证结项）

本目录用于直接对原厂量产固件 `C3M1生产烧录.hex` 做最小化二进制 Patch，**不改动任何原厂 UI、曝光、按键和镜头参数**，用于隔离验证并修复真机的 DNG 断层与左右镜像问题。

> **实机验证状态：全部通过（本目录工作已结项，后续新功能开发转入上层开源源码工程 `c3m1-firmware/`）**
> - **原厂片内备份校验**：从实机读出的 64 KB 备份 `c3m1_board_backup.bin` 与官方 `C3M1生产烧录.hex`（`0x08000000..0x0800E8C4`，59,588 B）**0 字节差异**（SHA-256 `d7b56c0cad6433c6...` 完全一致），且 `0x0800E8C4..0x08010000` 全为 `0xFF`。
> - **第 1 步（`C3M1_patched_dng_tear.hex`）**：使用此前复现 `CDI_456.DNG` 断层的同一张慢速 SD 卡实测，**画面错位拼接割裂彻底解决**。
> - **第 2 步（`C3M1_patched_dng_tear_orient.hex`，推荐日常刷入版）**：实机验证**左右镜像彻底修复、标准 `Orientation=1` 正向输出，且 `CFAPattern=GRBG` 色彩完全正常**。

## 目录内容

### 一、正式验证产物与备份
- `C3M1生产烧录.hex`：官方未改动量产固件备份（Patch 输入 + 一键回滚）
- `c3m1_board_backup.bin`：通过 ST-Link 从实机导出的 64 KB 原始 Flash 备份（与 `C3M1生产烧录.hex` 逐比特一致）
- `trampoline_dng_lock.s`：20 字节 Thumb-2 锁帧前置跳板（注入在 `0x0800E900`）
- `patch_dng_tear.py`：独立补丁脚本（默认出第 1 步纯净断层版；加 `--fix-mirror` 出第 2 步最终版；均**不改动任何 SDIO 逻辑**）
- **第 1 步（已通过）：`C3M1_patched_dng_tear.hex` / `.bin`** — 纯净 DNG 断层修复版（正文仅改 4 字节调用 + 20 字节跳板）
- **第 2 步（已通过，推荐最终版）：`C3M1_patched_dng_tear_orient.hex` / `.bin`** — 断层修复 + 硬件正向化修复左右镜像（`COM3=0xD0` + `Orientation=1` + `CFAPattern=GRBG`，正文共改 10 字节 + 20 字节跳板）

### 二、归档保留（`archive/`，未应用）：实验性 SDIO 稳定性补丁
> 说明：实拍样本 `CDI_456.DNG` 的断层已证实并实机验证为 AL422B 锁帧时序竞态所致，文件写入本身无 SDIO 错码；因此此前针对 STM32F103 ES0340 / 轮询中断保护的 SDIO 补丁已归档至 `archive/`，**不应用在正式验证固件中**。
- `archive/trampoline.s` / `archive/patch_factory_hex.py`：SDIO 轮询 NVIC 中断保护跳板与脚本
- `archive/C3M1_patched_sdio.hex` / `.bin`：归档的 SDIO 9 MHz 补丁版
- `archive/C3M1_patched_sdio_slow6mhz.hex` / `.bin`：归档的 SDIO 6 MHz 降频补丁版

---

## 最终版修复明细（`C3M1_patched_dng_tear_orient.hex`，均已反汇编与实机验证）

### 第 1 步：DNG 画面断层修复（`C3M1_patched_dng_tear.hex` 已包含）

| 补丁项 | 地址 | 原指令 | 修改后 | 修复原因 |
| :--- | :--- | :--- | :--- | :--- |
| **1. 拍照入口重定向** | `0x080016B6` | `bl 0x08000B98` (`ff f7 6f fa`) | `bl 0x0800E900` (`0d f0 23 f9`) | 在进入 `camera_save_dng()` 之前先执行锁帧等待 |
| **2. 锁帧前置跳板** | `0x0800E900..13` | `0xFF` 空闲 Flash（Page 29 尾部） | 轮询等待 `ov_sta (0x20001691) == 2`，随后 `bx 0x08000B99` 尾调用原 `camera_save_dng()` | 确保 `FIFO_WEN (PB3)` 先被 VSYNC 中断锁定为 `0`（完整一帧冻结在 AL422B 中），再执行 `f_open`、`f_write(592B 头)` 和会冻结 CPU/中断 20~40 ms 的 `STMFLASH_Write()`，杜绝慢卡把 Flash 擦除推入 `ov_sta == 1` 窗口导致第 2 帧在 384 KB AL422B 中回绕覆盖 |

### 第 2 步：DNG 左右镜像与硬件正向化修复（`C3M1_patched_dng_tear_orient.hex` 叠加）

原厂 RAW 初始化将 `COM3 (0x0C)` 设为 `0x50`（仅水平镜像，关闭垂直翻转），又在 DNG 文件头里用 `Orientation = 3`（旋转 180°）校正上下颠倒，结果水平镜像相消导致成片左右镜像。修复有两种思路：
- **仅改元数据**：保持传感器输出不变，只把 DNG 头 `Orientation` 从 `3` 改为 `4`（垂直翻转）；
- **硬件正向化（本补丁采用）**：直接把 RAW 模式 `COM3` 改为与取景预览一致的 `0xD0`（`VFLIP=1, HMIRROR=1`），同步将 DNG `Orientation` 改为标准正向 `1`、`CFAPattern` 改为 `GRBG`。

| 补丁项 | 地址 | 原值 / 原指令 | 修改后 | 修复原因 |
| :--- | :--- | :--- | :--- | :--- |
| **3. 开启 RAW 垂直翻转** | `0x08002632` | `movs r1, #0x50` (`50 21`) | `movs r1, #0xD0` (`d0 21`) | 让 RAW 模式 `COM3` 与取景预览（`0x0800254E`）一致：`VFLIP=1, HMIRROR=1`（硬件原生正向，消除左右镜像） |
| **4. DNG 方向改正向** | `0x080080C6` | `Orientation = 3` (`03 00 00 00`) | `Orientation = 1` (`01 00 00 00`) | 硬件已出正向图像，标准 `Orientation = 1` 所有看图/后期软件零兼容问题 |
| **5. 同步 Bayer 排列** | `0x0800811A` | `CFAPattern = BGGR` (`02 01 01 00`) | `CFAPattern = GRBG` (`01 00 02 01`) | `VFLIP=1` 倒序读取 480（偶数）行将 `BGGR` 垂直翻转为 `GRBG`（实机实拍已确认色彩完全正常） |

> **为什么选择“硬件正向化”，而不是“仅改 `Orientation=4`”？**
> - **利于后续机内流式 JPEG**：STM32F103 仅有 64 KB SRAM，后续在机内从 AL422B 按条带流式编码 VGA JPEG 时无法整帧缓存再垂直翻转。改为 `COM3=0xD0` 后，传感器直接从左上角正向输出，编出的 JPEG 与 DNG 原生就是标准 `Orientation=1`，彻底避开部分看图器/社交软件忽略 EXIF 翻转标志的问题。
> - **零画质与算力代价**：机内 3×3 去马赛克支持 `GRBG` 只是 2-bit 行列奇偶索引常量，零额外开销；后续若做 OV7725 片内 1D Log/Gamma 压扩（`GAM1~15` + `LinearizationTable`），属于逐像素标量映射，也完全不受 Bayer 排列变化影响。

## 重新生成命令

```bash
cd c3m1-factory-patch

# 第 1 步：纯净 DNG 断层修复版
python3 patch_dng_tear.py

# 第 2 步（推荐最终版）：断层 + 左右镜像（硬件正向化）修复版
python3 patch_dng_tear.py --fix-mirror
```

## SWD 烧录与回滚（实机验证命令）

> **安全与接线提醒**：
> 1. 主板 MicroUSB (`USB1`) 的 `VBUS` 直连 `VCC3.3`，**严禁直插电脑 5V USB**！供电二选一（关机由调试器 `3.3V` 供，或开机电池自供电且悬空调试器 `3.3V`）。
> 2. **切勿使用 OTG MicroUSB 线**：OTG 线头内部将 `Pin 4 (ID)` 短接至 `GND`，会把主板 `NRST` 持续拉死在 0V 导致无法 halt/读写 Flash。使用普通 4 芯非 OTG 数据线（`VBUS=3.3V, D-=SWDIO, D+=SWCLK, GND=GND`，不接 `NRST`）并配合 `reset_config none` 即可稳定烧录。

```bash
cd c3m1-factory-patch

# 推荐：使用 ST-Link 烧录最终验证版（断层 + 左右镜像修复，不擦除 0x08070000 快门计数）
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg \
  -c "reset_config none; program C3M1_patched_dng_tear_orient.hex verify reset exit"

# 如仅需第 1 步纯净断层版
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg \
  -c "reset_config none; program C3M1_patched_dng_tear.hex verify reset exit"

# 一键回滚至原厂未改动出厂固件
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg \
  -c "reset_config none; program C3M1生产烧录.hex verify reset exit"

# 如使用 CMSIS-DAP，将 `-f interface/stlink.cfg -f target/stm32f1x.cfg` 替换为 `-f ../dap.cfg`
```

---

## 附录：归档备用的 SDIO 补丁明细（位于 `archive/`，未应用）

| 补丁项 | 地址 | 原指令 / 原值 | 修改后 | 设计目的 |
| :--- | :--- | :--- | :--- | :--- |
| **1. 关闭 SDIO 硬件流控** | `0x080018F6` | `mov.w r3, #0x4000` (`4f f4 80 43`) | `movs r3, #0; nop` (`00 23 00 bf`) | 规避 STM32F103 勘误 ES0340 |
| **2. 轮询读写屏蔽外设中断** | `0x08005C52`<br>`0x08005C76`<br>`0x0800E900..3F` | `bl 0x080042F4`<br>`bl 0x08004548` | 读写期通过 `NVIC->ICER[0..1]` 屏蔽外设中断并保存/恢复 `NVIC->ISER[0..1]`（保留 `SysTick`） | 防止轮询塞数期被中断打断导致 SDIO FIFO 欠载/溢出 |
| **3. 放宽卡就绪等待超时** | `0x08007468`<br>`0x0800749C` | `100,001` (`0x000186A1`) | `1,000,000` (`0x000F4240`) | 兼容慢卡内部擦写停顿 |
| **4. 可选降频（`slow6mhz`）** | `0x080018FC` | `movs r3, #6` (9 MHz) | `movs r3, #10` (6 MHz) | 降速对照测试 |
