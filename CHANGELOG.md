# Changelog

All notable changes to `c3m1-firmware` are documented in this file.

## [v0.1.4] - 2026-11-25

详细发布说明见 [docs/releases/v0.1.4.md](./docs/releases/v0.1.4.md)。

### Added
- **索尼微单风格常驻 1px 峰值对焦（`PEAK COLOR`）**：永远开启，菜单支持 `RED / YELLOW / WHITE / BLUE` 选色与色块预览；采用水平 1px NMS 细丝 + 随 ISO 自适应阈值 + `s_lut_g6` 双行乒乓零拷贝加速，`1X` / `4X` 均生效。
- **64 档实时迷你直方图（`HISTOGRAM`）**：菜单可开关，右下角内联渲染 `66×26 px` 暗底直方图与 1/4 刻度线，完整覆盖 `0..63` 满量程亮度，高光截断 >2% 时右侧亮红警示。
- **半透明三分构图线（`GRID 3X3`）**：菜单可开关，在 `1X` 取景模式叠加 50% 半透明 1px 三分线，`4X` 放大自动隐藏。
- **配置结构 `schema v2`**：`0:CAYE_CFG/c3m1.cfg` 保持 32 字节定长并持久化 `peak_color`、`hist_en`、`grid_en`。

## [v0.1.3] - 2026-11-25

详细发布说明见 [docs/releases/v0.1.3.md](./docs/releases/v0.1.3.md)。

### Added
- **纯英文菜单系统（8 项）**：长按 `ASA` ≥ 600 ms 进入（无 SD 卡拒绝进入），波轮跨 `15↔0` 滚动、`SHOT` 切换/确认、`ASA` 返回；支持 `MAX AUTO ISO`、`MIN AUTO SHT`（4 段 P 曲线衔接慢门）、`ANTI-FLICKER`（50/60Hz/OFF）、`BATTERY DISP`（VOLT/PCT）、`FILM LEVER`（ON/OFF）、`FORMAT SD`（`FF_USE_MKFS=1` 零额外 RAM 机内格式化）与 `SYSTEM INFO`，底栏常驻版本号与 Git 短哈希（`-dirty`）。
- **SD 卡二进制配置固化（`0:CAYE_CFG/c3m1.cfg`）**：32 字节二进制格式，开机/热插卡执行长度、魔数、结构版本、固件版本号、CRC-32 与字段范围 6 重校验，并持久化全部菜单项与上一次关机的 ISO 模式。
- **白玉楼制作所 0.5s 开机动画**：仅占 ~1 KB Flash / 0 B 静态 RAM（徽标归白玉楼制作所所有；「白玉楼制作所」字模采用开源字体 Google Noto Sans CJK SC DemiLight / SIL OFL 1.1），后半段与 SD/传感器初始化并行，显示完整版本号与短哈希。
- **构建期版本号与短哈希注入**：`Makefile` 自动生成 `build/fw_version.h`，并在 CI 中启用 `fetch-depth: 0`。

### Changed
- 拆分 [BUILD.md](./BUILD.md) 与 [README.md](./README.md)，在 `README.md` 新增独立菜单说明与素材/字体许可声明。
- 移除所有快门计数清零逻辑，保持与原厂一致的只增不减计数行为。

## [v0.1.2] - 2026-11-25

详细发布说明见 [docs/releases/v0.1.2.md](./docs/releases/v0.1.2.md)。

### Added
- **OV7725 片内 10-bit Gamma 压扩 + 310 KB 自动还原 DNG**：开启 `COM7=0x01 / DSP_Ctrl1=0x87 / DSP_Ctrl4=0x02`，改为紧凑 8-bit 封装并内嵌 12-bit 精度 `LinearizationTable`（Tag `0xC618`, `0..4064`），体积较原厂 618 KB 减半、写卡提速 2×，同时暗部灰阶精细度提升至 2.62×。
- **OV7725 硬件 ABLC 抗温漂与分通道动态黑电平**：开启 `COM13=0xE6` 彻底消除热机高 ISO `+120` LSB 洋红暗部雾霾，并按当前增益动态写入分通道 `GRBG BlackLevel` 消除极端拉曝光暗部偏绿。
- **实验报告与分析脚本归档**：新增 `docs/ov7725_experiments.md` 及 `scripts/analyze_gamma_exp.py`、`scripts/analyze_blc_exp.py`，并记录 Type-C `CC1/CC2` 共用单颗 `5.1k` 充电避坑说明。

## [v0.1.1] - 2026-11-25

详细发布说明见 [docs/releases/v0.1.1.md](./docs/releases/v0.1.1.md)。

### Added
- 新增 GitHub Actions CI 自动编译工作流（`.github/workflows/ci.yml`），对 `main` 分支推送与 PR 自动校验构建。
- 新增标签触发自动构建与 Pre-release 发布工作流（`.github/workflows/release.yml`），自动读取 `docs/releases/<tag>.md` 并附带 CI 编译体积与 SHA-256 校验值上传 `.hex` / `.bin` 产物。

### Changed
- 从 `main` 分支清理 `releases/v0.1.0/` 中已跟踪的二进制文件，后续所有发行产物统一由 CI 发布至 GitHub Release Assets；发布说明归档至 `docs/releases/`。

## [v0.1.0] - 2026-11-25

首个完成全链路实机验证的开源基础发行版，详细发布说明与烧录校验见 [docs/releases/v0.1.0.md](./docs/releases/v0.1.0.md)。

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

## [v0.0.1] - 2026-11-25

原厂二进制固件定向补丁结项版本（[c3m1-factory-patch/README.md](./c3m1-factory-patch/README.md)），针对原厂 `C3M1生产烧录.hex` 做最小化二进制补丁并完成实机验证。

### Added
- 新增原厂量产机 64 KB Flash 实机备份（与官方生产 hex 比特级一致）及自包含二进制补丁脚本与产物（`c3m1-factory-patch/`）。
- 统一补丁与自研基线的默认日期约定：FAT 文件日期 `2026-12-26`，DNG 内嵌 `DateTime` 为 `2026:08:17 00:00:00`。

### Fixed
- **DNG 慢卡断层撕裂修复**（`C3M1_patched_dng_tear.hex`）：前置 `ov_sta == 2`（锁死 `FIFO_WEN = 0`）后再写 SD 卡与 Flash，实机验证彻底消除 AL422B 回绕拼接断层。
- **DNG 左右镜像修复**（`C3M1_patched_dng_tear_orient.hex`）：将 OV7725 RAW 改为 `COM3 = 0xD0` 硬件正向，同步调整 DNG `Orientation = 1` 与 `CFAPattern = GRBG`，实机验证色彩与方向均正常。
