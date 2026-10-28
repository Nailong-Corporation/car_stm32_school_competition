# A14：灰度接口改为向车头的弯排针

本版从 A13 复制，保留原 A13。J10 从 1×10 2.54 mm 排母改为 [XFCN PZ254R-11-10P / C492418](https://item.szlcsc.com/502974.html) 1×10、2.54 mm 方针弯排针，安装在 PCB 顶面，针脚向 `FRONT`（板坐标 +Y）水平伸出。PCB 丝印为 A14；原有 J10 焊孔中心、板框以及其他器件和电源方案保持。

以顶视图、FRONT 向上为准，J10 从左到右为 `S7 S6 S5 S4 S3 S2 S1 S0 GND +3V3`，对应原灰度丝印 `8 7 6 5 4 3 2 1 GND VCC`。1 脚为 +3V3、2 脚为 GND、3～10 脚依次为 S0～S7；线路与 A13 一致。请用与排针适配的 2.54 mm 母端线束并逐脚核对，不要以传感器左右顺序替代电气针序。

弯针原封装的针脚伸出轮廓移到文档层，避免印制丝印伸出板框；针序和 FRONT 提示移至不会被排针本体遮挡的位置。按器件库轮廓计算，前向针端轮廓约超出板框 **2.0 mm**，插接头与线束还需要额外空间。这是库模型的名义几何，不是实物或外壳碰撞验证，下单与装配前应检查灰度传感器线束、外壳和车头空间。

## 已完成的验证

- 嘉立创保存并重新打开 A14 后，原理图与 PCB 同步，严格 PCB DRC 为 0，见 [pcb-drc.json](pcb-drc.json)；原理图 DRC 保留 A13 既有 U4 `ATTRMATCHCODE` 警告，见 [schematic-drc.json](schematic-drc.json)。
- A13 与 A14 的 35 个器件、149 个引脚网络完全一致；J10 的 10 个焊孔中心未变，旋转为 180°，库封装的针端方向映射为 +Y。见 [j10-verification.json](j10-verification.json)。
- 修整 J10 1 脚周围顶、底地铜避让；10 个焊盘到最近异网顶层铜的距离均符合至少 0.1524 mm 检查，其中 +3V3 焊盘约 0.304 mm，见 [J10-clearance.json](J10-clearance.json)。独立铜几何检查为 48 网连通、0 异网短接，见 [copper-verification.json](copper-verification.json)。
- Gerber 和钻孔的 100 × 90 mm 板框、四个安装孔、100 个连接器孔、两组 15.24 mm 插座排距与 46 个贴片焊盘中心，以及 35 个器件的 BOM 和坐标已检查，见 [manufacturing-verification.json](manufacturing-verification.json)。

## 文件

- [A14 原生工程](stm32PCB/stm32PCB.eprj3) 和 [源工程 ZIP](A14-native-project.zip)
- [A14 打样 Gerber 与钻孔](A14-prototype-Gerber.zip)
- [原生 BOM](A14-BOM.csv)、[贴片 BOM](A14-SMT-BOM.csv)、[坐标文件](A14-PickAndPlace.csv)：原生导出为 UTF-16 制表符；贴片 BOM 仍包含直插件，下单时按装配方式筛选。
- [顶视预览](A14-PCB-preview.png)、[原理图网表](A14.netlist.json)与校验脚本。

A14 继承 A13 的电机电源：J8.1 直接接电池正极，绕过板上 F1/D1，核心板仍由板载 5 V 降压供电。三串锂电满充约 12.6 V、TB6612 VM 工作上限 13.5 V；电机回灌/瞬态、堵转电流与温升、保护板特性及三组 3.3 V 接口总负载仍须在首次带载前确认。没有烧录、上电、实际插装或实车测试。A14 与 A13 的 Gerber、BOM 和坐标不可混用。
