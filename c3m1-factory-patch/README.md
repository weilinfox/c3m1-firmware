# C3M1 原厂固件二进制补丁（SDIO 写卡稳定性修复验证）

本目录用于直接对原厂量产固件 `C3M1生产烧录.hex` 做最小化二进制 Patch，**不改动任何原厂 UI、曝光、按键和镜头参数**，专门用于隔离验证 SDIO 写卡稳定性问题。

## 目录内容

- `C3M1生产烧录.hex`：原厂未改动量产固件备份（既作为 Patch 输入，也用于一键回滚）
- `trampoline.s`：Thumb-2 汇编跳板源码（注入在 `0x0800E900`，共 64 字节，位于原固件末尾 `0x0800E8C4` 之后的空闲 Flash）
- `patch_factory_hex.py`：自动化二进制 Patch 脚本（先逐字节校验原固件字节，全部匹配才打补丁）
- `C3M1_patched_sdio.hex` / `.bin`：标准补丁版（维持原 `ClockDiv=6` 即 9MHz SDIO 时钟）
- `C3M1_patched_sdio_slow6mhz.hex` / `.bin`：降频保守版（`ClockDiv=10` 即 6MHz SDIO 时钟，用于排查挑卡/高速信号完整性）

## 具体修改点（均已通过 `arm-none-eabi-objdump` 反汇编复核）

| 补丁项 | 地址 | 原指令 / 原值 | 修改后 | 修复原因 |
| :--- | :--- | :--- | :--- | :--- |
| **1. 关闭 SDIO 硬件流控** | `0x080018F6` | `mov.w r3, #0x4000` (`4f f4 80 43`) | `movs r3, #0; nop` (`00 23 00 bf`) | 规避 STM32F103 官方硅片勘误 **ES0340**（开启硬件流控会导致 `SDIOCLK` 毛刺、写卡 CRC 错误与挑卡） |
| **2. 轮询读写屏蔽外设中断** | `0x08005C52`<br>`0x08005C76`<br>`0x0800E900..3F` | `bl 0x080042F4` (`HAL_SD_ReadBlocks`)<br>`bl 0x08004548` (`HAL_SD_WriteBlocks`) | 跳转至 `0x0800E900`/`0x0800E908` 跳板，读写期间保存并屏蔽 `NVIC->ISER[0..1]` 全部外设中断，保留 `SysTick` 计时，返回前恢复 | 防止高优先级 `VSYNC (EXTI9_5)` / 按键 / `TIM6` 打断轮询塞数导致 128B SDIO FIFO 下溢 (`TXUNDERR`) / 溢出 (`RXOVERR`) |
| **3. 放宽卡就绪等待超时** | `0x08007468`<br>`0x0800749C` | `100,001` (`0x000186A1`) | `1,000,000` (`0x000F4240`) | 避免部分 SD 卡内部 Flash 块擦除停顿 100~250ms 时误判超时返回 `RES_ERROR` |
| **4. 可选降频（仅 `slow6mhz` 版）** | `0x080018FC` | `movs r3, #6` (9 MHz) | `movs r3, #10` (6 MHz) | 针对劣质/挑卡 SD 卡或走线信号裕量不足的对照测试 |

## 重新生成命令

```bash
cd c3m1-factory-patch
python3 patch_factory_hex.py
# 或生成降频版：
python3 patch_factory_hex.py --clkdiv 10 --out C3M1_patched_sdio_slow6mhz.hex
```

## SWD 烧录与回滚

> **安全提醒**：主板 MicroUSB (`USB1`) 的 `VBUS` 直连 `VCC3.3`，严禁直插电脑 5V USB！请使用外部 CMSIS-DAP / ST-Link 通过 5 芯 SWD 转接线烧录。

```bash
cd c3m1-factory-patch

# 使用 CMSIS-DAP 烧录标准补丁版（复用仓库根目录的 dap.cfg）
openocd -f ../dap.cfg -c "program C3M1_patched_sdio.hex verify reset exit"

# 如需测试 6MHz 降频版
openocd -f ../dap.cfg -c "program C3M1_patched_sdio_slow6mhz.hex verify reset exit"

# 一键回滚至本目录备份的原厂未改动出厂固件
openocd -f ../dap.cfg -c "program C3M1生产烧录.hex verify reset exit"

# 如使用 ST-Link 而非 CMSIS-DAP，将 `-f ../dap.cfg` 替换为：
# openocd -f interface/stlink.cfg -f target/stm32f1x.cfg -c "program C3M1_patched_sdio.hex verify reset exit"
```
