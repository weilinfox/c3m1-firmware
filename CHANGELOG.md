# Changelog

All notable changes to `c3m1-firmware` are documented in this file.

## [v0.1.8] - 2026-11-26

详细发布说明见 [docs/releases/v0.1.8.md](./docs/releases/v0.1.8.md)。

### Added
- **软件时钟系统与两页式开机设时（`BOOT CLOCK`）**：开屏动画版本号后显示完整构建时间（`YYYY-MM-DD HH:MM:SS`）；菜单新增 `BOOT CLOCK`（`ON / OFF`，默认 `ON`），开启后每次开机分两页设定年月日（4 位年）与时分秒（波轮调数、快门确认下一项、`4X` 返回上一项，首次取构建时间、后续取 SD 卡保存时间，关闭时按构建时间运行）；实时驱动 FatFs `get_fattime()`、DNG `DateTime` 及直出 JPEG EXIF `APP1`，并在开机确认、拍照后、进出休眠与配置保存时自动持久化至 SD 卡（休眠期间同步按秒计时）。
- **低功耗呼吸灯休眠（`AUTO SLEEP`）与手动休眠**：菜单新增 `AUTO SLEEP`（`5M / 10M / 15M / 20M / 25M / OFF`，默认 `10M`）；休眠时关闭背光、ST7789（`SLPIN`）、OV7725（软休眠）、`ADC1` 与 HSE/PLL 降频至 8 MHz HSI + `__WFI()`，以 100 Hz Gamma 2.0 PWM 驱动 `LED1` 做 3 秒周期呼吸灯；短按 `ASA` 秒醒并自动吞掉唤醒按键。
- **硬件电源树文档**：在 `README.md` 与 `docs/architecture.md`（§3.10）详细记录主板无法真断电软关机的硬件原因与休眠关机操作建议。

### Changed
- **顶板长按交互重映射**：长按 `4X`（≥ 600 ms）改为进入菜单，长按 `ASA`（≥ 600 ms）改为手动进入休眠并保存当前时间（方便拨断物理开关 `SW4` 前落盘时间）。
- **曝光补偿收敛为精确 `±2.0EV`**：`EXP COMP` 调整为 `-2.0EV ~ +2.0EV`（13 档 `1/3 EV`，弹窗标尺改为宽距 `-2..0..+2`），与 OV7725 相对 18% 中灰 `+2.43 EV` 的真实上限严格一致。
- **配置结构升级 `schema v10`（40 B）**：新增持久化 `auto_sleep`、`boot_clock` 与 `saved_epoch`。

## [v0.1.7] - 2026-11-26

详细发布说明见 [docs/releases/v0.1.7.md](./docs/releases/v0.1.7.md)。

### Added
- **全屏 4:3 无裁切取景（`320×240`）与半透明内联 HUD**：取景覆盖全部 480 行 RAW，底部 20 px 区域改为 25% 压暗半透明随行合成（零闪烁），`4X` 放大与 `GRID 3X3` 水平线同步居中校准。
- **三种测光模式（`METER MODE`）与中心测光取景框**：菜单 `EXP COMP` 下方新增 `METER MODE`（`STD / CENTER / ETTR`，`schema v7`）；`CENTER` 采用中央 6× 权重（中心占比 67%，取消全局高光拉低钳制）并在画面叠加 75% 亮白四角框与中心十字指示；`ETTR` 以第 98 百分位高光为目标向右曝光。
- **回看 18-bit RGB666 满宽显示与快速 DNG 放大**：回看切换至 ST7789V3 `COLMOD=0x66` + 2×2 Bayer 有序抖动并铺满 `320 px` 全宽，新 DNG 预览图无除法 2× 缩放，DNG 1:1 放大改用无分支快速 8-bit 路径。

### Fixed
- **P 档 AE 收敛提速与 `ETTR` 过曝回拉修复**：改用事件驱动 1 帧稳定等待、大偏差 `±2.0 EV`/步快速收敛，并基于过曝像素比例加速 `ETTR` 过曝回拉。
- **手动快门切回 `AUTO`（如 `1/8000s → AUTO`）过冲修复**：切入 `AUTO` 快门/ISO 时按当前等效物理曝光量无缝映射至 P 曲线，且 AE 等待切档后的 2 帧脏帧排空，彻底消除切档爆白与快门冲顶回落。
- **构建零警告**：新增 `-Wl,--no-warn-rwx-segments` 消除 GCC 15 链接器 RWX 段提示，实现 0 编译警告 + 0 链接警告。

## [v0.1.6] - 2026-11-25

详细发布说明见 [docs/releases/v0.1.6.md](./docs/releases/v0.1.6.md)。

### Added
- **机内照片回看（Playback）与 1:1 放大平移**：过片扳手（`FILMKEY/PF10`）支持短拨上弦 / 长按 ≥ 600 ms 进入回看（永远从最新一张开始，自动合并同编号 `DNG+JPG`，复用 `11.3 KB` 条带缓冲实现 `.bss` 零新增）；1X 态支持波轮循环翻页、优先 65 ms 极速显示 DNG 嵌入 `160×120` 预览图、自动兼容历史无预览老 DNG 与纯 Baseline JPEG（内置纯整数流式 `c3m1_jdec` 解码器）；`4X` 进入 1:1 像素放大（DNG 直解全尺寸 Bayer RAW，JPG 按 ROI 跳过解码，右上角带迷你视口框，波轮上下 / `ASA` 左 / 快门右平移，再按 `4X` 退回 1X）；1X 态短按 `ASA` 弹出二次确认删图窗（同步删除同名 `.DNG` 与 `.JPG`），短拨过片扳手退出回看。
- **底部状态栏 0.1 EV 实时测光表**：基于每帧线性光均值按纯整数 `log2(eff_g / 46)` 在 ISO 与白平衡之间实时显示 `-5.0EV ~ +3.0EV` 曝光偏差（手动 M 档与自动档均生效，白/黄/红/青多色状态提示）。
- **±3 EV（1/3 EV 步进）曝光补偿（`EXP COMP`）**：菜单第 3 项新增 `EXP COMP`（`-3.0EV ~ +3.0EV` 共 19 档），按快门弹出微单风格 `-3..0..+3` 刻度标尺窗（波轮左右微调、`4X` 一键归零、快门保存、`ASA` 取消），直接驱动 Q4 精度软件闭环 AE。
- **调试信息开关（`DEBUG INFO`）与独立 FPS 徽标**：菜单 `FORMAT SD` 与 `SYSTEM INFO` 之间新增 `DEBUG INFO`（`OFF / ON`，默认 `OFF`），将帧率移出底栏；开启后在直方图正上方内联零闪烁显示完整 `FPS: xx`。
- **配置结构 `schema v6`**：`0:CAYE_CFG/c3m1.cfg` 保持 32 字节定长，新增持久化 `debug_info` 与 `ev_comp`。

### Fixed
- **底栏高速自动快门截断修复**：扩宽底栏快门字段至 10 字符（60 px），完整显示 `SA:1/23000` / `SA:1/11500` 并重排 320 px 状态栏间距。
- **SD 热插卡配置保护加固**：热插卡挂载前暂停 OV7725 FIFO 写入以防中断干扰 SDIO，并区分 `CFG_LOAD_IO_ERR`（自动延时重试一次，读错时绝不覆写卡内原有有效配置）。

## [v0.1.5] - 2026-11-25

详细发布说明见 [docs/releases/v0.1.5.md](./docs/releases/v0.1.5.md)。

### Added
- **DNG 嵌入预览图（`IFD0` + `SubIFD`）**：遵循标准 DNG 1.4 规范在 `IFD0` 写入 `160×120` 校准 sRGB 缩略图（~57.6 KB，复用写卡缓冲零额外 RAM），`SubIFD` 保持全尺寸 `644×480` 8-bit 自动线性化 Bayer RAW，兼容 macOS 访达预览、Adobe Camera Raw 及后续机内快速回看。
- **纯整数流式 VGA Baseline JPEG 直出与同帧双出（`FILE FORMAT`）**：新增无浮点条带编码器（`18×644 B` 行带 3×3 双线性去马赛克 + 12-bit 线性扣黑 + 3×3 色彩矩阵 + IJG 整数 DCT，输出 `640×480` YCbCr 4:2:0 Q90 JPEG）；菜单新增首项 `FILE FORMAT`（`DNG` / `DNG+JPG` / `JPG`，默认 `DNG`），`DNG+JPG` 复用 AL422B `/RRST` 实现严格同帧双出。
- **白平衡预设（`WHITE BAL`）与底栏实时色温显示**：菜单第二项新增 `WHITE BAL`（按色温升序排列：`AUTO` / `3200K` 灯光卷 / `5500K` 普通胶片 / `6500K` 日光卷，默认 `AUTO`），同步作用于取景、DNG `AsShotNeutral` 与直出 JPEG；底部状态栏第三项新增实时白平衡色温显示（自动模式白字 `A5500K`，固定色温黄字 `3200K`/`5500K`/`6500K`）。
- **自动 ISO 上限扩展至 `3200`（`MAX AUTO ISO`）**：新增 `3200` 档位（对应 OV7725 硬件最高 `31×` 增益），底栏自动 ISO 1/3 档读数同步扩展至 `3200`。
- **配置结构 `schema v4`**：`0:CAYE_CFG/c3m1.cfg` 保持 32 字节定长，新增持久化 `save_mode` 与 `wb_mode`。

### Fixed
- **统一 OV7725 DNG 与直出 JPEG 色彩管线**：修正原厂倒置的 DNG `ColorMatrix1`，改用经 Bradford 色适应校准的 `D65` `ColorMatrix1` + 动态写入 `AsShotNeutral (0xC628)`，并与 JPEG 编码器 3×3 CCM 严格对应，使 Adobe Camera Raw「原照设置」与机内直出 JPEG 白平衡及色彩高度一致。

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
