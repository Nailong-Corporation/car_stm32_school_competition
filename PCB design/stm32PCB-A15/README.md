# A15：R3 和 D1 基础库换料评审版

本版从 A14 独立复制，保留 A14。用户确认接受 R3 从 13.3 kΩ 改为 13 kΩ、标称约 5.18 V，并表示核心板 5 V 输入允许最高至少 5.41 V。A15 已完成原理图、PCB、Gerber 与 BOM 的换料核对；**没有进行电源实测或实物装配，Gerber 先标为评审用途。**

## 换料与电气影响

| 位号 | A14 | A15 | 影响 |
| --- | --- | --- | --- |
| R3 | 13.3 kΩ ±1%，0603，C25952，扩展库 | 13 kΩ ±1%，0603，C22797，基础库 | R2 保持 100 kΩ，理论输出由约 5.077 V 升至约 5.181 V；无极性器件的 1/2 编号与原件互换，PCB 已旋转保持 GND 与 BUCK_FB 的物理连接。 |
| D1 | SS54-C、SMC、C397626，扩展库 | SS54、SMA、C22452，基础库 | 新器件 1 脚 A → DC_FUSED、2 脚 K → VIN_PROTECTED；原件编号相反，已核对阴阳极及网络。SMA 与 SMC 封装不同，已替换实际焊盘并调整附近电池直供走线和地铜。 |

依据 [TI TPS54302 数据手册](https://www.ti.com/lit/ds/symlink/tps54302.pdf) VIN=12 V 下反馈电压 0.581/0.596/0.611 V，按 R2=100 kΩ ±1%、R3=13 kΩ ±1% 作独立静态极限组合，`Vout = VFB × (1 + R2/R3)`：典型约 **5.1806 V**，计算极限约 **4.9617～5.4059 V**。这不是实测值，未计输出纹波、过冲、瞬态、线损或其他影响；用户确认的至少 5.41 V 上限距静态计算上限约 4 mV，不代表动态余量。上电前核实核心板及连接到 5 V 的全部外设允许范围；首次上电先断开模块，用限流电源测量空载/负载、启动过冲和纹波。

A15 保留 A14 顶部向车头弯排针和三组 3.3 V 排针。J10 名义针端轮廓约越过前板边 2 mm，线束空间及实物适配仍未验证。电机 VM 直接接电池正极，绕过板上 F1/D1；三串锂电满充约 12.6 V，TB6612 VM 工作上限为 13.5 V，回灌尖峰、负载与温升仍未验证。

## 验证

- 保存、关闭并重新打开后，严格 PCB DRC 0 项；原理图 DRC 保留 A14 既有 U4 `ATTRMATCHCODE` 警告。
- 原理图与 PCB 35 个器件、149 个引脚网络一致；除 D1 与 R3 引脚编号按新器件调整外，其余器件逐脚网络与 A14 一致，见 [electrical-verification.json](electrical-verification.json)。
- 独立铜几何检查：48 个网络连通、无异网短接；D1 焊盘到最近异网顶层铜约 0.304 / 0.566 mm，R3 焊盘最小约 0.249 mm，见 [D1-R3-clearance.json](D1-R3-clearance.json)。
- Gerber 板框、100 个连接器孔、46 个贴片焊盘中心、BOM、坐标及原生工程 ZIP 已检查，见 [manufacturing-verification.json](manufacturing-verification.json)。此类检查不代表工厂贴装成功或温升/动态电气合格。

## 文件

- [原生工程](stm32PCB/stm32PCB.eprj3) · [工程 ZIP](A15-native-project.zip)
- [评审 Gerber/钻孔](A15-REVIEW-ONLY-Gerber.zip) · [原生 BOM](A15-BOM.csv) · [贴片 BOM](A15-SMT-BOM.csv) · [坐标](A15-PickAndPlace.csv)
- [PCB 预览](A15-PCB-preview.png) · [网表](A15.netlist.json) · 检查脚本和 JSON 报告

贴片 BOM 包含直插件，实际下单按装配方式筛选。基础库类别、库存和收费应在下单时重新确认。没有烧录、上电、实物适配、负载或实车测试；不要混用 A14/A15 的制造文件。
