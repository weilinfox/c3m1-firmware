# C3M1 原厂固件二进制补丁（单变量实机验证）

本目录用于直接对原厂量产固件 `C3M1生产烧录.hex` 做最小化二进制 Patch，**不改动任何原厂 UI、曝光、按键和镜头参数**，用于隔离验证真机问题。

## 目录内容

### 一、分阶段验证目标（推荐按顺序验证）
- `C3M1生产烧录.hex`：原厂未改动量产固件备份（Patch 输入 + 一键回滚）
- `trampoline_dng_lock.s`：20 字节 Thumb-2 锁帧前置跳板（注入在 `0x0800E900`）
- `patch_dng_tear.py`：独立补丁脚本（默认出第 1 步纯净断层修复版；加 `--fix-mirror` 出第 2 步叠加镜像修复版；均**不改动任何 SDIO 逻辑**）
- **第 1 步：`C3M1_patched_dng_tear.hex` / `.bin`** — 纯净 DNG 断层修复固件（全固件仅改 4 字节调用 + 20 字节跳板，先用慢卡单变量验证断层是否消失）
- **第 2 步：`C3M1_patched_dng_tear_orient.hex` / `.bin`** — 第 1 步通过后再验证：在断层修复基础上采用**方案 2**修复 DNG 左右镜像（`COM3=0xD0` 硬件正向 + `Orientation=1` + `CFAPattern=GRBG`）

### 二、归档保留（`archive/`，当前未应用）：实验性 SDIO 稳定性补丁
> 说明：实拍样本 `CDI_456.DNG` 的断层已证实为 AL422B 锁帧时序竞态所致，文件写入本身无 SDIO 错码；因此以下此前针对 STM32F103 ES0340 / 轮询中断保护的 SDIO 补丁已归档移入 `archive/` 目录，**不应用在 `C3M1_patched_dng_tear.hex` 中**，待慢卡确实暴露写卡报错/卡死时再另行验证。
- `archive/trampoline.s` / `archive/patch_factory_hex.py`：SDIO 轮询 NVIC 中断保护跳板与脚本
- `archive/C3M1_patched_sdio.hex` / `.bin`：归档的 SDIO 9 MHz 补丁版
- `archive/C3M1_patched_sdio_slow6mhz.hex` / `.bin`：归档的 SDIO 6 MHz 降频补丁版

---

## DNG 断层纯净修复点（`C3M1_patched_dng_tear.hex`，已反汇编核对）

| 补丁项 | 地址 | 原指令 | 修改后 | 修复原因 |
| :--- | :--- | :--- | :--- | :--- |
| **1. 拍照入口重定向** | `0x080016B6` | `bl 0x08000B98` (`ff f7 6f fa`) | `bl 0x0800E900` (`0d f0 23 f9`) | 在进入 `camera_save_dng()` 之前先执行锁帧等待 |
| **2. 锁帧前置跳板** | `0x0800E900..13` | `0xFF` 空闲 Flash（Page 29 尾部） | 轮询等待 `ov_sta (0x20001691) == 2`，随后 `bx 0x08000B99` 尾调用原 `camera_save_dng()` | 确保 `FIFO_WEN (PB3)` 先被 VSYNC 中断锁定为 `0`（完整一帧冻结在 AL422B 中），再执行 `f_open`、`f_write(592B 头)` 和会冻结 CPU/中断 20~40 ms 的 `STMFLASH_Write()`，杜绝慢卡把 Flash 擦除推入 `ov_sta == 1` 窗口导致第 2 帧在 384 KB AL422B 中回绕覆盖 |

### 第 2 步叠加修复点（`C3M1_patched_dng_tear_orient.hex`，方案 2 正向化）

| 补丁项 | 地址 | 原值 / 原指令 | 修改后 | 修复原因 |
| :--- | :--- | :--- | :--- | :--- |
| **3. 开启 RAW 垂直翻转** | `0x08002632` | `movs r1, #0x50` (`50 21`) | `movs r1, #0xD0` (`d0 21`) | 让 RAW 模式 `COM3` 与取景预览（`0x0800254E`）一致：`VFLIP=1, HMIRROR=1`（硬件原生正向，消除左右镜像） |
| **4. DNG 方向改正向** | `0x080080C6` | `Orientation = 3` (`03 00 00 00`) | `Orientation = 1` (`01 00 00 00`) | 硬件已出正向图像，标准 `Orientation = 1` 所有看图/后期软件零兼容问题 |
| **5. 同步 Bayer 排列** | `0x0800811A` | `CFAPattern = BGGR` (`02 01 01 00`) | `CFAPattern = GRBG` (`01 00 02 01`) | `VFLIP=1` 倒序读取 480（偶数）行将 `BGGR` 垂直翻转为 `GRBG`；若实拍色彩偏品红/绿，说明该批次硅片内部已补偿行奇偶，用 `--cfa bggr` 重生即可 |

## 重新生成命令

```bash
cd c3m1-factory-patch

# 第 1 步：重新生成纯净 DNG 断层修复版（不应用 SDIO 补丁）
python3 patch_dng_tear.py

# 第 2 步：重新生成 断层 + 左右镜像（方案 2 正向化）验证版
python3 patch_dng_tear.py --fix-mirror

# 若第 2 步实拍偏品红/绿，切换回 BGGR 重生：
# python3 patch_dng_tear.py --fix-mirror --cfa bggr
```

## SWD 烧录与回滚

> **安全提醒**：主板 MicroUSB (`USB1`) 的 `VBUS` 直连 `VCC3.3`，严禁直插电脑 5V USB！请使用外部 CMSIS-DAP / ST-Link 通过 5 芯 SWD 转接线烧录。

```bash
cd c3m1-factory-patch

# 1. 第 1 步验证：烧录纯净 DNG 断层修复版（用同一张慢卡做单变量对照测试）
openocd -f ../dap.cfg -c "program C3M1_patched_dng_tear.hex verify reset exit"

# 2. 第 2 步验证：第 1 步通过后，烧录 断层 + 左右镜像修复版
openocd -f ../dap.cfg -c "program C3M1_patched_dng_tear_orient.hex verify reset exit"

# 3. 一键回滚至本目录备份的原厂未改动出厂固件
openocd -f ../dap.cfg -c "program C3M1生产烧录.hex verify reset exit"

# 如使用 ST-Link 而非 CMSIS-DAP，将 `-f ../dap.cfg` 替换为：
# openocd -f interface/stlink.cfg -f target/stm32f1x.cfg -c "program C3M1_patched_dng_tear.hex verify reset exit"
```

---

## 附录：保留备用的 SDIO 补丁明细（未应用在 `C3M1_patched_dng_tear.hex` 中）

| 补丁项 | 地址 | 原指令 / 原值 | 修改后 | 设计目的 |
| :--- | :--- | :--- | :--- | :--- |
| **1. 关闭 SDIO 硬件流控** | `0x080018F6` | `mov.w r3, #0x4000` (`4f f4 80 43`) | `movs r3, #0; nop` (`00 23 00 bf`) | 规避 STM32F103 勘误 ES0340 |
| **2. 轮询读写屏蔽外设中断** | `0x08005C52`<br>`0x08005C76`<br>`0x0800E900..3F` | `bl 0x080042F4`<br>`bl 0x08004548` | 读写期通过 `NVIC->ICER[0..1]` 屏蔽外设中断并保存/恢复 `NVIC->ISER[0..1]`（保留 `SysTick`） | 防止轮询塞数期被中断打断导致 SDIO FIFO 欠载/溢出 |
| **3. 放宽卡就绪等待超时** | `0x08007468`<br>`0x0800749C` | `100,001` (`0x000186A1`) | `1,000,000` (`0x000F4240`) | 兼容慢卡内部擦写停顿 |
| **4. 可选降频（`slow6mhz`）** | `0x080018FC` | `movs r3, #6` (9 MHz) | `movs r3, #10` (6 MHz) | 降速对照测试 |
