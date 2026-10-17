# STM32 载板 A7：U4 焊盘修正、基础库按键与核心板方向复核

2026-10-17，在嘉立创 EDA 专业版中完成 A7。基于 A6 的 C5 / C6 / L1 碰撞修复，修正 U4 焊盘，并按用户确认将 SW1 换为基础库 1.5 mm 矮按键。核心板 40 个引脚丝印已对照用户实物照片逐一核查。原生工程入口为 [stm32PCB/stm32PCB.eprj3](stm32PCB/stm32PCB.eprj3)，旧修订保留。

**验证状态：严格 PCB DRC 为 0；原理图仍有一条 U4 的 `ATTRMATCHCODE` 属性匹配警告，原因尚未确认。** 已独立核对 U4 料号、引脚网络、实际焊盘和导出 Gerber，但未将此警告关闭或豁免，不能描述为原理图 DRC 全通过。本目录制造包供重新评审，尚未提交订单。

## 本次修改

| 位置 | A7 实际修改 | 核查依据 |
| --- | --- | --- |
| U4 / TPS54302DDCR / C311983 | 单焊盘 0.60 × 1.10 mm；同排节距 0.95 mm；相对两排中心距 2.70 mm；工程封装名统一为 `TSOT-23-6` | [TI TPS54302 数据手册](https://www.ti.com/lit/ds/symlink/tps54302.pdf)，PDF 第 30 页 DDC0006A 推荐焊盘图 |
| U4 周边 | 重接新焊盘，局部 VIN 短铜路宽度由 1.0 改为 0.9 mm，保留 3.1 mm 电源主段；更新顶层地铜避让 | 原规则不变，严格 PCB DRC 和独立铜几何检查 |
| SW1 | 旧 C381041 / TS-1102S-C-B-B 换为 **C318884 / TS-1187A-B-A-B**；本体 5.1 × 5.1 × 1.5 mm，用户已接受原 4.5 mm 高按键改为 1.5 mm | [JLCPCB 官方料页](https://jlcpcb.com/partdetail/C318884)当日显示 Basic；[厂家图纸存档](references/SW1-C318884.pdf) |
| SW1 焊盘及连接 | 四焊盘 1.00 × 0.75 mm，左右中心距 6.00 mm，上下中心距 3.75 mm；1/2 接 BTN，3/4 接 GND，按下连通两组 | 厂家推荐布局与电路图；原理图及 PCB 逐脚核对 |
| 版本丝印 | `REAR / STM32 CAR A7` | 原有用户文字保留 |

U4 采用工程内嵌本地封装，修改已写入原理图与 PCB 的本地库记录，没有改全局系统库。旧 3D 模型仍为保守高度模型，不能据此声称已完成全板三维装配核查。SW1 沿用库封装标题中的 `P3.70`，实际焊盘排距已按厂家推荐焊盘图调整为 3.75 mm；尺寸以上表、原生文件和 Gerber 为准。EDA 保存后重新生成的原理图 Channel ID 已同步至 PCB，电路连接未改变。

A6 中 L1 左移约 4.01 mm 的修复保留。C5 / C6 与 L1 的名义本体间距约 2.55 mm，按厂家最大尺寸包络约 2.30 mm；计算范围及公差依据见 [A6 说明](../stm32PCB-A6/README.md)。A7 没有再移动这三个器件。

## 核心板丝印方向

核查对象为 [用户核心板实物照片](reference-core-board.jpg)，以核心板元件面向上、载板顶面朝向观察者、`FRONT ^ GRAY 3.3V` 在上方为准：**USB 朝左，SWD 朝右**。翻到载板底面观察时，视觉上的左右会镜像，不能沿用顶视图直接判断。

上排 J6，从左到右：

```text
GND GND 3V3 NRST PB11 PB10 PB1 PB0 PA7 PA6 PA5 PA4 PA3 PA2 PA1 PA0 PC15 PC14 PC13 VBAT
```

下排 J7，从左到右：

```text
PB12 PB13 PB14 PB15 PA8 PA9 PA10 PA11 PA12 PA15 PB3 PB4 PB5 PB6 PB7 PB8 PB9 5V GND 3V3
```

以上 40 个标签均与照片及对应焊盘网络一致，标签横向位置也逐一对齐。J6 顶视从左到右为引脚 20→1，J7 为 1→20；实际针距 2.54 mm，两排中心距 16.002 mm。原有方向标记正确，因此保留 `USB <` / `SWD >`。这证明照片对应的引脚顺序与 PCB 丝印相符，实物排母插入深度、模块底部间隙和接口可达性仍待新板试插。

可查看 [顶面丝印预览](PCB-top-silkscreen-A7.png)及[整板铜路预览](PCB-board-A7.png)。整板图从顶面观察双层铜，不是底面镜像焊接图。

## 基础库替换结果

完整清单见 [parts-review.md](parts-review.md)。当日实时 EDA 查询共 19 类 / 32 个载板器件，A7 为 **7 类基础库、12 类扩展库**；其中扩展库贴片器件从 7 类减少为 6 类，减少的是 SW1。其余扩展库贴片器件为 R3、C11、U4、D1、F1、L1，保留原规格。另 6 类为通孔连接器，仍保留在完整 BOM 中。

本次检索未找到可以确认直接替换其余器件的同规格基础库料。尤其没有为降低料费改变反馈电阻、电容或电感电流规格；基础库 SMA 封装 SS54 也没有当作原 SMC 封装 SS54-C 的直接替料。基础/扩展分类和可供货状态以后续实际 SMT 选料页为准，本文是 2026-10-17 查询快照。

## 验证与文件

- 严格 PCB DRC 返回空数组，规则与 A6 相同，见 [pcb-drc.json](pcb-drc.json)。原理图警告原文见 [schematic-drc.json](schematic-drc.json)。
- 关闭 A7 后从磁盘重新打开，严格 PCB DRC 仍为 0；32 个器件的位置、旋转、封装及料号、143 个焊盘和全部丝印与导出快照一致，见 [reopened-check.json](reopened-check.json)。
- 原理图网表、PCB 的 32 个器件 / 143 个焊盘网络一致，涉及 48 个网络；独立铜几何确认所有网络连通、无异网短路，6 个 VIN 并联过孔均双面接入。见 [verification.json](verification.json)及[copper-verification.json](copper-verification.json)。
- Gerber 的 U4 六焊盘、SW1 四焊盘及 L1 两焊盘坐标和尺寸与 PCB 一致；导出的全部 32 行贴片坐标与工程一致。原生 BOM 和标准字段 SMT BOM 均为 19 组 / 32 件、料号一致。
- 工程 ZIP 文件集合及逐文件内容与磁盘原生工程相同；板框为 100 × 90 mm，安装孔、插座孔径及电源过孔也从制造数据复核。

| 文件 | 用途 |
| --- | --- |
| [stm32PCB-A7-project.zip](stm32PCB-A7-project.zip) | 完整可编辑工程；解压后打开 `stm32PCB/stm32PCB.eprj3` |
| [stm32PCB-A7-REVIEW-ONLY-Gerber.zip](stm32PCB-A7-REVIEW-ONLY-Gerber.zip) | 本修订制造数据，供工厂重新评审 |
| [stm32PCB-A7-SMT-BOM.csv](stm32PCB-A7-SMT-BOM.csv) | UTF-8 BOM 编码，逗号分隔；标准字段 Comment / Designator / Footprint / Quantity / LCSC Part #，封装栏使用供货封装名称，完整保留通孔连接器 |
| [stm32PCB-A7-BOM.csv](stm32PCB-A7-BOM.csv) | EDA 原生 BOM，保留工程封装名称；UTF-16、制表符分隔 |
| [stm32PCB-A7-PickAndPlace.csv](stm32PCB-A7-PickAndPlace.csv) | EDA 原生贴片坐标，UTF-16、制表符分隔，mm 单位；包含通孔器件，后续装配选择需按实际工艺处理 |
| [stm32PCB-A7.netlist.json](stm32PCB-A7.netlist.json) / [pcb-state.json](pcb-state.json) | 原理图存档网表和 PCB 核查快照 |

复核入口：

```sh
python3 hardware/stm32PCB-A7/verify_design.py
# 需要已有 Shapely 2.x 环境
python3 hardware/stm32PCB-A7/verify_copper.py
```

尚未完成新 PCB 实物试插、全板三维装配、贴装样板、电源负载/温升或实车验证。此前 J13 槽孔与现行厂家推荐图的差异仍保留，见 [原 BOM 核查](../../docs/BOM_5406283A_Y1-review.md)。U4 原理图属性匹配警告也仍需处理。没有重新下单、烧录或操作硬件；`hardware_tested=false`、`fabrication_release_approved=false`。后续评审使用本目录同一 A7 的 Gerber、BOM 和坐标，旧订单实际使用的 Gerber 仍未确定。
