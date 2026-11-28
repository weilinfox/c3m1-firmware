# Changelog

All notable changes to `c3m1-firmware` are documented in this file.

## [Unreleased]

### Added
- **全模态电源看门狗（开机设表 / 菜单 / 回看）与坏卡/无卡分级菜单**：`c3m1_power_service_modal()` 覆盖开机两页设表、主菜单及全部子弹窗、照片回看与删图弹窗，自动执行 `< 3.20V` 强制低电休眠与 `AUTO SLEEP` 息屏（开机设表休眠唤醒完整保留走时且不误写旧快照）；取消未挂载 SD 卡拒进菜单限制，插卡但未格式化/损坏时可进入菜单执行 `FORMAT SD`（先自动重初始化 SDIO）救卡，无卡时亦可查看 `SYSTEM INFO`、切换 `COUNTER FMT` 及临时调参（`RAM ONLY` 提示、不触碰 FatFs）。
- **4-bit SDIO 升级 `DMA2_Channel4` + 关闭 `HWFC` 并提频至 `24.0 MHz`（`4b6d48c`；独立屏幕压测工具留档于 `ea5c62c` / `5ea5156`）**：彻底解决原轮询模式与 STM32F103 ES0340 §2.8.1 `HWFC` 勘误在 18 MHz 下的读写失败，绕过 STM32F1 `HAL_DMA_IRQHandler` 遗留 `DMA_CCR_EN=1` 导致半双工写方向切换被硬件忽略的 HAL 缺陷，并补齐 RX 末块 CRC 竞态防护与 512 B 非对齐中转；`MX_SDIO_SD_Init()` 自动按 **`24 MHz (0x01) → 18 MHz (0x02) → 12 MHz (0x04) → 9 MHz (0x06)`** 阶梯执行双次多块 DMA 读取 + FNV-1a 校验降频，并在 `SYSTEM INFO` 新增 `SD BUS` 行直观展示当前速率与降频回退状态；实机三卡（SanDisk Ultra 16GB SDHC、TF 转接 Lexar GOLD 128GB UHS-II、Fanxiang K520 128GB UHS-II）热插拔多轮复测全档位 100% PASS，24 MHz 较 9 MHz 裸读提速 **+45%~+55%**、FatFs 读提速 **+34%~+46%**、FatFs 写提速 **+19%~+25%**。
- **生产态串口日志编译门控（`C3M1_DEBUG_LOG`）**：`Makefile` 新增 `DEBUG_LOG ?= 0`（默认关闭串口 TX 与 `printf`，省 `~6.6 KB` Flash、`huart1`/newlib 缓冲并消除阻塞毛刺；调试可随时 `make DEBUG_LOG=1` 一键恢复）。

### Changed
- **取景热路径、LCD 流式绘制与全屏半透明定格预览**：`raw_demosaic_1x1_to_buf` 改 2× 展开实现 **0 字节栈帧 / 0 寄存器溢出**（`-418 B` Flash）；`vf_chroma_vig_pack_rgb565` 改单寄存器 `g_q8` 消除内循环溢出；`lcd_show_char` 与 `lcd_draw_rectangle` 升级窗口流式 FSMC 写并修复 `y0 + size == 240` 底边截断；`ov7725_camera_refresh` 拆分为 3 个 `noinline` 子函数（`vf_render_1x_aa / 1x_raw / 4x`）；快门定格升级为全屏 `320×240` + 50% 半透明 `SAVE` 底栏与 3×3 构图线；顶部 HUD 扩宽至 `104 px` 防止大容量卡长模式标截断 `BW/BW+/BWR`。
- **HDR / JPEG / 回看性能与体验全面提速**：HDR Pass 1 升级 `4,096 B` 扇区对齐 + `256 B` carry 至 Pass 2（全片 `165 × 4,096 B` 零半扇区写）+ 免费水平 AA + Pass 2 复用空闲 `scratch` `5–8` 行对批读；JPEG 编码消除 `/ 15L` 除法、4 px 暗角步进并改单行 `cb_row/cr_row`（降栈 `-224 B`）；灰度 `640×480` JPEG 1X 回看补齐 4 行条带 DMA 快路径；回看扫描按 `(date_packed, num, variant)` 排序并按日期降序剪枝旧目录；菜单大跨度参数（光圈/畸变/暗角）支持快速拧动 5× 加速、未改值确认免写 SD 卡并全处补齐 `ASA` 长按退回；`FILINFO` 与 `c3m1_cfg` `FIL` 复用于 `work_buf` / `s_work_buf` 顶端（回看/配置降栈 `572–616 B`）；去重 `c3m1_dng.c` 静态表与 `c3m1_wb_gains_to_kelvin()`。

### Fixed
- **根治暖光 / `3200K` 高光青斑与 `B&W RED` 亮部封顶不足**：单帧 JPEG、取景 LUT 与 4X DNG 回看统一按 `min(wb_r, 256, wb_b)` 每帧一次归一白点（零热循环除法、`5500K/6500K` 逐位一致），饱和高光稳定达纯白 `RGB=(255,255,255)`。
- **修复 `H16` `BaselineExposure` 截断、包围快门时序与旧版 DNG 回看一致性**：`BaselineExposure` 钳位由 `±5.00 EV` 拓宽至 `±7.00 EV`，`H16` `ref_white` 锚定中帧 `ratio_q8[1]`；慢门 HDR/AEB 禁用首帧 `CLKRC` 切换并修正手动 `0 EV` 子帧 EXIF；HDR/AEB/JSR 与慢门路径完整传播 VSYNC 超时；单张存盘改为写完并 `f_close` 成功后再提交快门计数，失败自动 `f_unlink` 清除残档；低电休眠回电切普通休眠唤醒后强制重挂防换卡残留；峰值对焦垂直阈值对称化；旧版无缩略图 DNG 1X/INFO 回看同步支持黑白影调与 16-bit HDR 映射。

## [v0.1.15] - 2026-11-28

详细发布说明见 [docs/releases/v0.1.15.md](./docs/releases/v0.1.15.md)。

### Added
- **全链路机内黑白胶片模式（`COLOR MODE`，`schema v19` 复用最后 1 B 保持 48 B、0 B `.bss` 增长）**：菜单第 1 项新增 `COLOR MODE`（`COLOR / B&W / B&W HI / B&W RED`），`FILE FORMAT` 顺延至第 3 项；取景器采用 64 级严格中性 RGB565 胶片 S 曲线查表（零偏绿、保留彩色对焦峰值与后台 AWB/AE，左上角叠加 `BW/BW+/BWR`）；直出 JPEG 跳过彩色 CCM、线性光正权重混色并编码标准单通道灰度 Baseline JPEG（`SOF0 Nf=1`，S 曲线前执行保颗粒柔和亮度 NR）；DNG 保持 100% 无损彩色 Bayer RAW 并写 IFD0 `0x010E` 影调标记与黑白嵌入缩略图，回看 1X/4X 自动按单文件还原对应黑白影调与单通道高感降噪。
- **内部 Flash 快门计数页（`0x08070000`）磨损均衡与 `COUNTER FMT` 菜单开关**：支持原厂单槽（`STOCK`）与 1,022 槽顺序追加写（`WEAR-LVL`，页尾 `"C3WL"` 魔数自动识别，寿命 > 1,000 万次，翻页先写 `slot[0]` 再写魔数防断电丢计数）热切换；`STOCK → WEAR-LVL` 零擦除原地升级、`WEAR-LVL → STOCK` 还原 100% 原厂单槽布局；移除 `stmflash.c` 中 `g_flashbuf[1024]` 净省 **`2,048 B` `.bss`**，补充全路径 `HAL_FLASH_*` 返回值校验、即时回读验证与 `0xFFFF` 空槽哨兵防护，`SYSTEM INFO` 同步显示格式与已用槽位。
- **亏电休眠 60 Hz 走时与回电 `≥ 25%` 自动切回普通休眠**：亏电休眠改用 8 MHz HSI + 60 Hz `TIM6`（`+10 ppm`）驱动双闪并持续推进软件时钟（期间不做 SD 热插拔检测，交由切入普通休眠后再监测）；每 3 s 以 `~1.1 ms` 零 PLL 8 MHz HSI ADC 采样（`adc_battery_sample_hsi8_mv()`）巡检轻载电量，连续 2 次（6 s）达到 **`≥ 25%`** 即在息屏状态下零闪烁、零递归无缝切回普通呼吸灯休眠（覆盖开机、运行与唤醒全部 3 个入口）。

### Changed
- **休眠轻载电量曲线分段实测重标定（实验五增补至 24 组稳定工作→休眠数据对）**：实测确认高电端（`≥ 3.91V`）与低电端（`≤ 3.45V`）压差为 `+50 mV`、中段放电平台（`3.50–3.84V`）稳定收窄为 `+30 mV`；将 `adc_mv_to_percent_light()` 由固定扣减 `50 mV` 升级为 11 点分段轻载曲线（`3300 / 3430 / 3530 / 3580 / 3620 / 3660 / 3710 / 3770 / 3850 / 3960 / 4050 mV`），消除中段休眠速览 `-3%~-5%` 偏低跳变，全段实测误差 `≤ ±1%`（中位 `0%`）。
- **取景与全屏回看底栏通透化、去顶线与 3×3 构图线全屏贯通**：取景底栏（`y = 220..239`）与全屏回看（1X `CLEAN`/`CLIP`、4X `ZOOM`，含 DNG 18-bit RGB666 与 JPEG RGB565 全路径）统一升级为全屏 `320×240` + **50% 半透明暗底**内联叠加（移除顶部分割横线，1X 抗锯齿覆盖底栏 20 px、回看底栏提示字提亮至浅银 `0xD69A` 保可读，复用 `s_osd_bar_buf` 保持 **0 B `.bss` 增长且峰值栈严守 `< 6 KB`**）；取景 `3×3` 九宫格竖线延伸至屏幕最底端 `y = 239` 并先绘于底栏之下实现自然半透明遮盖。
- **标配 `6mm F1.6` 镜头预设暗角定标（实验 7）**：经白屏垫纸漫射全对焦行程平场测试及叠合 `WarpRectilinear (-0.072)` 验证，将 `6mm F1.6` 预设的 `.vignette` 由 `0` 更新为 **`23`（`+1.15`）**。
- **1X 实时取景融合 64 级暗角优化（回收 `1,568 B` `.bss`）**：将暗角增益直接代数融合进解马赛克 `vf_chroma_vig_pack_rgb565`（单周期 `MUL` + `USAT ASR`）与黑白 S 曲线前缩放，彻底移除第二遍 `vf_apply_vig_row_be/le` 后处理与 `1.5 KB` 暗角查表；实机验证 `LENS VIG = +1.15` 下 1X 取景由 `8 fps` 恢复至 **`9–10 fps`**，且暗角过渡由 16 档提升至 64 级。

## [v0.1.14] - 2026-11-27

详细发布说明见 [docs/releases/v0.1.14.md](./docs/releases/v0.1.14.md)。

### Added
- **统一抗锯齿管线（`schema v18`，48 B 无损迁移，`ANTI-ALIASING` 默认 `ON`）**：1X 实时监看与快门定格预览在 `INCAM DISTORT = OFF / ON` 两条路径上统一启用 U1 行间 50/50 混色（峰值对焦仅叠于 `y0` 保持 1 px 锐线）；DNG `160×120` IFD0 缩略图升级 `4×4` 线性光全区域盒式下采样；直出 VGA JPEG 常规路径升级 `20` 行滑动条带 + Malvar-He-Cutler 5×5 梯度修正解马赛克，显著消除斜线拉链锯齿与红蓝彩边。
- **直出 JPEG 三级 ISO 自适应高感降噪（`HIGH ISO NR` 默认 `ON`，0 B `.bss` 增长）**：覆盖畸变矫正开/关全路径，纯整数零除法零软浮点；`ISO ≤ 200`（100/125/160/200）或关闭时 100% 旁路（逐字节一致）；`ISO 250–320` 仅轻度色度联合双边；`ISO 400–640` 优先画质保留 Malvar 5×5 解马赛克并叠加 8-bit Gamma 域热/冷像素剔除 + 查表色度/暗部增强亮度双边 + 融合 DCT AC 柔缩；`ISO 800–3200` 切换平滑 3×3 双线性并随 ISO 渐进加强。
- **DNG 回看机内高感降噪（受 `HIGH ISO NR` 与 DNG EXIF `ISO` 控制，不改动磁盘 DNG）**：复用共享 `scratch` 尾部实现 `1X` 缩略图 3×3 联合双边与 `4X` 1:1 放大（畸变/非畸变、8-bit/16-bit HDR 全覆盖）8-bit 热像素剔除 + 2×2 5-tap 色度联合双边 + 3-tap 暗部增强亮度双边，全程保持 18-bit RGB666 + 2×2 抖动 DMA1 输出。

### Changed
- **1X 监看全组合稳跑 `9–10 fps` 与回看热路径提速**：以 Thumb-2 `LDRD/STRD` 8 像素展开重写 `vf_avg_row_be()`（~2.99 ms/110 行）、`scratch` 第 3 槽 `peak_g[2]` 实现 `y0` 原地零 `memcpy` DMA，配合 `vf_warp_row_1x()` 11 节点水平对称与 0 栈溢出 4 px 循环（`-648 B` Flash、省 `~4 ms`），实机验证 `INCAM DISTORT (ON/OFF) × ANTI-ALIASING (ON/OFF)` 全部 4 种组合均稳跑 **`9–10 fps`**（`4X` 维持 `8 fps`）；DNG 4X 回看内联 `cam12→sRGB8`、`pb_chroma_row_t` 连续排布、单周期 `__SSAT`/`__USAT` 融合 RGB666 抖动并恢复 8-bit 畸变 16 行批量读卡。
- **链接脚本栈下限加固**：`STM32F103ZETX_FLASH.ld` `_Min_Stack_Size` 由 `0x1000`（4 KB）提升至 `0x1800`（6 KB），实测 JPEG/回看全链路峰值栈均严守 6 KB 红线以内。

## [v0.1.13] - 2026-11-27

详细发布说明见 [docs/releases/v0.1.13.md](./docs/releases/v0.1.13.md)。

### Added
- **全链路机内镜头矫正与 `LENS PROFILE` 预设（`schema v16/v17`，48 B 无损迁移）**：新增 `LENS PROFILE`（`UNKNOWN` / `6mm F1.6`，自动填充参数并写 EXIF `LensModel`）与 `INCAM DISTORT` 开关（默认 `OFF` 仅写 DNG `OpcodeList3`；开启后同步驱动 1X 实时监看、快门定格预览帧、直出 VGA JPEG 单遍双线性采样与 DNG/HDR 嵌入缩略图）；DNG 回看 `4X` 放大直接从文件自身 SubIFD `OpcodeList3 (0xC74E)` 纯整数解析 `FixVignetteRadial` + `WarpRectilinear`，以 35 行水平裁剪环形缓存实时还原 8-bit 与 16-bit HDR DNG 的畸变和暗角。
- **运行态 `DRIVE MODE`（`SINGLE / BURST / AEB 3F`）**：支持按住快门连续连拍与免倒计时 3 张独立 `0EV / -2EV / +2EV` 包围曝光。

### Changed
- **取景器、存卡、回放与 Flash/栈第一梯队性能优化**：
  - 严守 AL422B 非并发帧锁（`g_vsync_state == 2`）下完成纯软件热循环提速（16 段径向暗角 LUT、小端 `vf_ring` + `-O3` 行指针外提 `vf_warp_row_1x`、4X 4B 对齐 `s_raw_row_buf[648]` + 4× 展开 `fifo_read_bytes` + 32-bit `raw_demosaic_1x1_to_buf`、默认关闭 1 Hz 阻塞串口打印），非零暗角下实测稳跑 **`1X 9–10 fps / 4X 8 fps`** 且全程零错色；
  - VGA JPEG 编码器全文件 `-O2` + 自然序 Q16 倒数量化表（0 除法）+ `__builtin_clz` + 全内联；DNG Pass 1 按 `4,096 B` 对齐切分 + Pass 2 借用 `s_bayer_strip` 按 `30,720 B` 批量落盘（`DNG+JPG` 单拍由 `5–6 s` 降至稳定 **`5 s`**）；
  - 回放升级 `c3m1_jdec` `-O2` + `2048 B` `inbuf` + `stream_getc` 内联快路径、640×480 JPEG `320×8` 条带 DMA、DNG 1X/INFO/4X 批量读 + 18-bit RGB666 DMA1 双缓冲、以及 JSR 目录单遍 `f_opendir/f_readdir` 扫描；
  - 移除未用 `asc2_3216`（`-6 KB` Flash）、纯整数 IEEE-754 编解码清零 `__aeabi_d*/f*` 软浮点库、`hsram1` 局部化（`-72 B .bss`）、HDR `FIL tf[3]` 挪至 `scratch` 尾部（`-1,776 B` 峰值栈）。
- **低时滞快门与电量曲线对齐**：`KEYSHOT` 改前沿瞬时触发 + 渲染中段早退，按下快门先绘精确锁存定格帧再写卡；工作态电量 `0%` 对齐 `3.25V`（`3.25–4.00V`），休眠轻载曲线统一 `+50 mV` 偏移（`3.30–4.05V`）。

### Fixed
- **根除 AL422B 边写边读偶发蓝色错色**：对照 `AL422B.pdf` 与主板原理图确认 `/RE` 接地、128 B 缓存与 `C34 = 1 µF` 硬件约束，关闭 RWW 恢复严格 `g_vsync_state == 2` 锁帧后错色 100% 消失。
- **四模型交叉代码审计全量加固**：修复 `BOOT CLOCK = OFF` 与热插卡时钟恢复、`c3m1_cfg_load` 纯只读 + 低电禁写修复、`c3m1_jdec` `DRI/RSTm` 与 IDCT 中间溢出防护、`SD_DATATIMEOUT = 2000 ms` + `HAL_GetTick()` 超时、VSYNC 原子状态机与 `SENSOR TIMEOUT!` 保护、`TIM6` 按需启停、`USART1` 纯 TX 防悬空 RX、链接脚本 `448 KB` Flash 边界保护、以及 4X 畸变环形缓存防超前驱逐与外部畸形 DNG 边界校验。

## [v0.1.12] - 2026-11-27

详细发布说明见 [docs/releases/v0.1.12.md](./docs/releases/v0.1.12.md)。

### Added
- **真 16-bit 4 帧机内 HDR（`16B 4F`）与 `DEL HDR BRK`**：`HDR MERGE` 新增 `16B 4F`（`-3/-1/+1/+3 EV`，总跨度 64×，实测动态范围达 `14.13–14.60 EV`）；菜单新增 `DEL HDR BRK`（默认 `OFF`），开启后在 HDR / JSR+HDR 全部拍完合成完毕后自动删除 `_0.._3` 包围单帧。
- **`JSR BURST` 三脚架超分连拍与回看整组支持**：新增运行态 `JSR BURST`（`OFF / 7F / 14F / 30F`，不写配置），与 `HDR MERGE` 正交组合归档至 `CDIxxxxx_7/_14/_30/`，拍完亮帧弹窗引导拍摄同参数匹配黑场 `CDIyyyyy_dark.dng`；回看自动折叠每个 JSR 目录为 1 条，支持品红类型标签、`INFO` 亮帧/黑场/体积统计与 `DELETE JSR SET?` 整组删除。
- **镜头焦距 / 光圈 EXIF 与 DNG 1.4 `OpcodeList3` 镜头矫正（`schema v15`，44 B 无损迁移 v10–v14）**：新增 `LENS FOCAL`（`OFF`、`4mm~500mm` 39 档原始标称，默认 `6mm`，EXIF 写 `0x920A` + 官方 `9×` `0xA405`）、`LENS APERTURE`（`F0~F32.0` 步进 `0.1`，默认 `F0`，EXIF 始终写 `0x829D` 含 `F0 → 0/1`）、`LENS DIST`（`-0.200~+0.200` 步进 `0.002`，写入标准 `OpcodeList3 (0xC74E)` `WarpRectilinear` 及 4:3 最大内接无黑边 `maxfov` 缩放，C3M1 原装镜头甜点 `-0.072`）与 `LENS VIG`（`0~+2.50` 步进 `0.05`，写入 `FixVignetteRadial` 并同步补偿直出 JPEG 与 DNG 缩略图线性域）；回看 `INFO` 页同步展示光圈与原始/等效焦距。
- **工作态 / 休眠轻载双电量曲线**：基于 9 组实测压差标定工作曲线 `3.20–4.00V` 与休眠速览专属轻载曲线 `3.30–4.05V`，消除工作态与休眠速览间的百分比及颜色跳变。

### Changed
- **实验文档完善**：在 `docs/ov7725_experiments.md` 补齐实验三至实验六（HDR 动态范围、LCD 撕裂、双电量曲线、镜头畸变标定与 `maxfov` 对比），并清晰标注 `[实测]` / `[脚本统计]` / `[估算/理论]` 来源。

## [v0.1.11] - 2026-11-26

详细发布说明见 [docs/releases/v0.1.11.md](./docs/releases/v0.1.11.md)。

### Added
- **机内 12-bit / 14-bit 多帧流式 HDR 合成（`HDR MERGE`）**：菜单新增运行态 `HDR MERGE`（`OFF / 12B 2F / 14B 3F`，默认 `OFF`，不写配置），三脚架 3 秒可取消自拍倒计时，全程锁定当前 ISO 纯快门包围并保留同组单帧 `_0/_1[/_2].dng` 与合成片 `_hdr.dng`，快门计数按实际曝光帧数累加（`2F +2`、`3F +3`）。
- **DNG / JPEG 完整 EXIF 曝光参数写入与回看 `INFO` 展示**：统一写入 `ExifIFD`（快门 `ExposureTime`、`ISOSpeedRatings`、`ExposureBiasValue`、`DateTimeOriginal/Digitized`），HDR 单帧 `_0/_1/_2` 自动叠加实际包围 EV 偏移；回看 `INFO` 页新增 `EXP` 行显示快门、ISO、EV 与白平衡色温。
- **回看跳过 HDR 包围单帧（`PB SKIP BRK`，`schema v11`）**：菜单 `BOOT CLOCK` 下方新增持久化开关（默认 `OFF`，无损兼容旧 `schema v10` 配置），开启后若同组存在 `_hdr` 则回看直接跳过 `_0/_1/_2`。
- **左上角剩余张数 / 拍摄模式透明 HUD 与新版色彩管线**：左上角内联叠加剩余可拍张数（`>99999` 5 位封顶）与拍摄模式（`DNG / D+J / JPG / H12 / H14`），快门计数移至 `DEBUG INFO`（`FPS:` / `SHT:` 双行对齐）；升级 Filmic S-Curve LUT、取景饱和度增益与鲜明 OV7725 CCM。
- **`< 3.20V` 低电强制休眠、带载电量曲线与休眠电量速览**：开机/运行/休眠唤醒三入口拦截 `< 3.20V` 低电并弹出 15 s 提示（`SWITCH OFF TO CHARGE`），息屏后 `LED1` 每 3 s 短双闪提示电源未断、按任意顶键复显 5 s 最轻载实时电压；百分比按 C3M1 ~400 mA 带载曲线标定为 `3.20V (0%) ~ 4.00V (100%)`（黄警对齐 `3.52V ≈ 25%`）；普通休眠按非唤醒键（`SHOT / 4X / 过片扳手`）在右下角显示最轻载电压与百分比 `3 s`（按 `ASA` 秒醒）。

### Changed
- **回看删图交互统一**：1X 全图与 4X 放大视图均改用过片扳手短拨触发删图确认弹窗。
- **文档补充**：`README.md` 与 `docs/architecture.md` 补充 TP4056 休眠充电安全分析、HDR 恒定 ISO 包围与 OV7725 时序要点、以及四档低电保护与带载电量标定说明。

### Fixed
- **根治 `4X` 放大下波轮锁死与按键漏触发**：在取景渲染循环每 16 行中段轮询顶板按键与 BCD 波轮（`~9–11 ms` 周期）并将 `DIAL_STALE_GAP_MS` 放宽至 `250 ms`。
- **修复 HDR 多项硬件与色彩缺陷**：修复 OV7725 向上增曝光 510 行过渡帧过曝、`DM_LN` 哑行提前锁存截断、以及白平衡高光未钳位导致的 HDR 1X/4X 回看过曝偏色与直方图未顶头问题。
- **修复低电菜单提示与休眠速览残影**：亏电阻止保存配置时菜单底栏改显红色 `LOW BAT`（不再误报 `SD ERR`）；`lcd_sleep_enter()` 进入 `SLPIN` 前先清黑 GRAM 且速览亮背光前等待 1 帧刷新，消除休眠首次速览的旧取景画面残影。

## [v0.1.10] - 2026-11-26

详细发布说明见 [docs/releases/v0.1.10.md](./docs/releases/v0.1.10.md)。

### Added
- **回看 `1X` 短按快门三态循环与索尼风格 `INFO` 详情页**：短按快门在「纯净 `1X` → `CLIP` 溢出标注 → `INFO` 详细信息」间循环切换；`INFO` 页左上角显示带红/蓝 `CLIP` 斜纹的 `160×120` 缩略图，右侧叠排 `Y / R / G / B` 四通道 64 档直方图（含网格线与边缘溢出提示），下方显示目录文件名、格式、EXIF/文件系统时间、分辨率/位深与白平衡色温、文件大小以及实时高光/暗部溢出占比与平均亮度。
- **照片按 8 位日期子目录（`CAYE_CAM/YYYYMMDD/`）存储与回看**：拍照按当前软件时钟自动建目录归档（`DNG+JPG` 同帧共享同一日期），回看自动跨日期子目录扫描并向下兼容旧版 `CAYE_CAM/` 根目录照片（索引容量 800 张，删空自动清理空日期目录）。

### Fixed
- **根治 16 档 BCD 波轮回看翻页跳两张与乱序**：`key.c` 改用单脚分时下拉采样消除 `R10=10k` 与四脚内部 `40k` 下拉在高位档（`7/11/13/14/15`）造成的电平跌落，并将消抖升级为 `55 ms` + 连续 `≥ 3` 次采样及渲染后自动重置窗口。
- **回看交互与边界加固**：`1X` 退回看及删图取消弹窗同步支持长按事件；`DNG+JPG` 部分删图失败仅清除成功侧标志位并提示 `PARTIAL DELETE!`；严格校验 EXIF `DateTime` 数字位以正确触发 FatFs 时间回退；`DNG+JPG` 中 `.dng` 损坏时自动回退显示 `.jpg`；开机设时消抖余量增至 `90 ms`；链接脚本 `_Min_Stack_Size` 提升至 `0x1000`。

## [v0.1.9] - 2026-11-26

详细发布说明见 [docs/releases/v0.1.9.md](./docs/releases/v0.1.9.md)。

### Added
- **回看 1X 过曝 / 过暗区域标注与 4X 过片手柄快捷退出**：回看 1X 全图（新 DNG 缩略图、老 DNG 解拜耳、纯 JPEG）以 45° 红斜纹标注过曝（`RGB ≥ 250`）、蓝斜纹标注死黑欠曝（`RGB ≤ 4`），4X 1:1 放大时不显示叠加；4X 放大态支持直接拨动过片手柄退出回看返回取景。
- **`README.md` 补充软件时间系统原理章节**：完整说明开屏构建时间、两页式开机设时、休眠走时与长按 `ASA` 休眠落盘时间流程。

### Fixed
- **跨版本升级保留 SD 卡用户配置**：`c3m1.cfg` 加载移除 `fw_major/minor/patch` 强一致拒载，只要 `schema_ver`（`v10`）与 CRC-32/字段范围合法即完整保留用户配置与保存时间。
- **修复开机设时年份偶发自跳**：`SET DATE (1/2)` 先执行 `key_init_state()` 与 50 ms BCD 拨盘消抖后再锁存初始拨盘位置。
- **修复慢门 `AUTO` 预览亮度脱节**：`MIN AUTO SHT` 低于 `1/50s`（`auto_aec > 500`）时在预览叠加 `GAIN` + 数字 LUT 补偿，消除暗光 AE 跑飞与成片过曝。
- **加固休眠唤醒、编解码器与 SDIO 边界**：唤醒前置 `HAL_ResumeTick()` 并检测休眠期 SD 拔/换卡；修复 `c3m1_jdec` Huffman Kraft 越界与非 `1×1` 灰度 MCU、DNG 解析 `uint32_t` 溢出与 `1×N` 下溢、小图放大迷你框越界、`c3m1_jpeg` `io_err` 短路、`sd_diskio` 拔卡快退及 `sdio` 死循环隐患。

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
