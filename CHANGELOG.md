# Changelog

All notable changes to `c3m1-firmware` are documented in this file.

## [v0.1.0] - 2026-11-25

首个完成全链路实机验证的开源基础发行版，详细发布说明与烧录校验见 [releases/v0.1.0/RELEASE_NOTES.md](./releases/v0.1.0/RELEASE_NOTES.md)。

### Added
- **常驻 RAW 微单化取景**：OV7725 全程常驻 `644×480` 8-bit `GRBG` RAW（`COM4=0x81` / 18 MHz / 45 fps），`1.5 KB` SRAM LUT + `DMA1` 双缓冲刷屏实现全 16 档 **15F** 取景与瞬时 `4X` 1:1 放大对焦。
- **零快门时滞与 50Hz 抗频闪 P 曲线 AE**：快门 `1/8000 ~ 1/50` 与 `AUTO`（`1/23000 ~ 1/50s`、ISO `100 ~ 1600`）零时滞拍摄，慢门 `1/25 ~ 2s` VSYNC 原子单帧长曝。
- **SDXC 原生支持（FAT32 + exFAT 至 2 TB）与 `/CAYE_CAM/` 目录**：升级至 FatFs R0.15a，修复 STM32F1 HAL SD `S18R` 握手与 2 TB 边界溢出，所有 DNG 自动建目录保存至 `/CAYE_CAM/CDIxxxxx.dng`。
- **安全 SD 热插拔与 `VREFINT` 自校准电池电压**：`SDIO_CLOCK_POWER_SAVE` + 拔卡 `DeInit` + 150 ms 消抖重挂；内部 1.20V 基准校准 `3.xxV` 三色显示与 `<3.25V` 低电写卡保护。
- **胶片式计数与 `LED1` 开漏状态反馈**：过片上弦推进待拍编号，`PD9` 开漏输出适配 `VCC5.0` 彻底消除取景微亮。

### Fixed
- 修复慢速 SD 卡写 DNG 时 AL422B FIFO 环形回绕造成的断层撕裂。
- 修复原厂 `COM3=0x50` + `Orientation=3` 导致的成片左右镜像，改用硬件正向 `COM3=0xD0` + `Orientation=1` + `GRBG`。
- 消除中帧重写 `CLKRC` 与 `COM4=0xC1` / `COM2=0x03` 振铃导致的偶发 1~2 行错色。
