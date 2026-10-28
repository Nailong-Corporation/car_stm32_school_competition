# STM32 小车载板 A13：首件打样版

从 A10 原生工程建立，保留原 A10。A13 已完成本次两项原理图及 PCB 修改，配套资料可用于首件 PCB 打样。**电池瞬态、驱动器电流和新增 3.3 V 负载仍缺少实测或器件资料；打样可制造不代表电源设计已通过实测，不应直接带电机上电。**

## 电源和接口

- 电池 J13.1 `DC_RAW` 直接连接 TB6612 模块 J8.1、C1.1 和 C2.1；J8.2 仍为 `+3V3`，地线共地。此电机支路接在 F1、D1 之前，**不经过板上保险和防反接**。原降压输出网络仍名为 `VM`，是约 5 V，并通过 R4 向核心板 `+5V` 供电；J2 仍是 5 V 输出，不能当作电池输入。
- 顶面右侧 J14/J15/J16 为 2.54 mm 两针排针，三组并联，顶视时各组左侧 1 脚为 `+3V3`，右侧 2 脚为 `GND`。3.3 V 来自核心板，并非独立稳压器。用户表示已有板带两个编码器正常，三组接口主要用于编码器，陀螺仪待定；尚未测量负载总电流与稳压器温升。
- 已知电池为三节 18650 串联，用户当前实测 11.6 V；按普通锂离子电芯 4.2 V 满充推算组电压 12.6 V，最终以电池和保护板规格为准。用户确认电机支持 12 V，每台堵转电流不超过 2 A。东芝手册给出 TB6612FNG 的 VM 工作上限 13.5 V；满电与工作上限之间仅 0.9 V，电机回灌、导线及开关尖峰尚未实测。单通道 2 A 堵转不能视作持续允许电流，实际驱动模块、堵转持续时间、限流和散热仍须核验。

## 验证结果

- 保存、关闭并重新打开 A13 后，严格 PCB DRC 为 0；最新丝印修改后再次检查为 0，见 [pcb-drc.json](pcb-drc.json)。原理图严格 DRC 仅有 A10 既有的 U4 `ATTRMATCHCODE` 警告，见 [schematic-drc.json](schematic-drc.json)；U4 贴片属性仍须按原 [警告说明](../../docs/U4-SMT-package-warning.md) 审核。
- 原理图网表与 PCB 共 35 个器件、149 个引脚网络一致。对照 A10，原器件网络变化仅为 C1.1、C2.1、J8.1 从 `VM` 改为 `DC_RAW`，新增 J14/J15/J16，见 [netlist-verification.json](netlist-verification.json)。
- 独立铜几何检查确认 48 个网络连通、未发现异网铜短接；对地铜执行避让并修复孤岛及 C2 地连接，见 [copper-verification.json](copper-verification.json)。这不等同于电流承载、回路电感或实物温升验证。
- Gerber 板框 100 × 90 mm，四个安装孔、100 个连接器镀通孔、两组 15.24 mm 排距及 46 个贴片焊盘中心已核对；六个新增排针镀通孔直径为 1.10002 mm。35 个器件的 BOM、坐标和原生工程 ZIP 与当前工程核对，见 [manufacturing-verification.json](manufacturing-verification.json)。PCB 顶视预览显示右侧原有竖排文字已移开新增接口，见 [A13-PCB-preview.png](A13-PCB-preview.png)。完整三维插装、线束及底盘干涉尚未验证。

- C1/C2 焊盘到异网铜的最小距离均不小于 0.25 mm，见 [local-clearance-verification.json](local-clearance-verification.json)。C2 地过孔已封阻焊；依据 Gerber 标称尺寸，其孔边到 C2 焊盘阻焊开窗保守间距约 0.229 mm，详见 [manufacturing-detail.json](manufacturing-detail.json)。
- 电池到驱动的新增供电主线约 1.524 mm 宽、99.86 mm 长。以 35 µm 铜厚、20 °C 粗算约 0.032 Ω；若两电机合计 4 A，线段约降压 0.13 V、耗散 0.52 W。这只是稳态估算，不代表经过验证的温升或额定电流。

## 文件

- [可编辑原生工程](stm32PCB/stm32PCB.eprj3)
- [首件打样 Gerber 和钻孔](A13-prototype-Gerber.zip)
- [贴片 BOM](A13-SMT-BOM.csv)沿用 A10 格式，包含待确认是否由工厂贴装的直插件；下单时按实际装配方式筛选。
- [原生 BOM](A13-BOM.csv)、[原生贴装坐标](A13-PickAndPlace.csv)：EDA 输出的 UTF-16 制表符文件；另有同名 `-UTF8.csv` 便于阅读。
- [原理图网表](A13.netlist.json)、[PCB 状态](pcb-state.json)、[逐脚核验脚本](verify_netlist.py)、[铜几何核验脚本](verify_copper.py)和[局部间距脚本](verify_local_clearance.py)（需 Shapely 2.x）、[制造数据核验脚本](verify_manufacturing.py)。

尚未完成模块型号和 3.3 V 外设负载核对、电池保护板的具体短路保护特性确认、电池满充时驱动端回灌/尖峰测试、堵转与热测试、机械实物适配。用户表示电池带保护板，但型号和保护特性未知，且不等同于板上电机支路经过 F1。没有下单、焊接或上电；不要将 DRC、网表及几何检查当作这些验证的替代。
