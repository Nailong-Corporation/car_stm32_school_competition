# STM32 载板 A10：放宽 C1/C2 附近的铜间距

> **2026-10-17 后续状态：** 用户确认本修订已下单并包含 SMT，1×20 排母换料已接受。[下单后电气终审](../../docs/A10-final-electrical-review.md)重新核查原生工程及制造包，未发现确定性电气接错；尚无上电、负载或温升实测。下文“没有下单”描述本修订生成时的历史状态，不代表当前订单状态。

2026-10-17，按用户截图及“调整接线”的要求，在 A9 副本上修整 TB6612 电源去耦区。C1 为 100 nF / 0603，C2 为 10 µF / 0805，均跨接 VM（5 V）与 GND。原 A9 保留；A10 的工程、Gerber、BOM 和坐标须配套使用。

## 本次修改

- C2 向左、向下各移动名义 **0.20 mm**，旋转、封装、料号和逐脚网络不变；同步连接 VM 的 **1.0 mm** 宽支路端点。
- 删除 C1/C2 附近 5 段绕近 VM 焊盘的旧 GND 细线，以重新生成的地铜连接这一区域。C1 的其余地回路保留，独立检查确认两颗电容地端均连通。
- 在顶层 `x=33…42 mm、y=27…34 mm` 的局部区域增大地铜避让至名义 **0.30 mm**；其他区域沿用原有约 0.25 mm 避让。坐标原点为板框左下角。
- AIN1 原路径及相邻控制线保留。检查发现直接横移 AIN1 会挤到 BIN1，采用移动 C2 和重接局部电源/地的方式增加余量，没有新增过孔或削细电源铜路。
- 版本丝印及原理图版本说明更新为 A10。STM32 / TB6612 的 15.24 mm 排距、A9 修正后的 J9 上排 / J8 下排针序、其他器件位置和固件保持原设计。

## 实际间距

以下为保存的 PCB 铜边缘距离，包含原生地铜的内孔，非器件中心距或实物测量：

| 检查位置 | A9 | A10 |
| --- | ---: | ---: |
| C2 的 GND 焊盘到 AIN1 | 0.153848 mm | **0.354508 mm** |
| C1 的 VM 焊盘到最近 GND 铜 | 0.156015 mm | **0.299638 mm** |
| C1/C2 所有 4 个焊盘到异网铜的最小值 | 0.153848 mm | **0.299638 mm** |
| C1 与 C2 最近焊盘间距 | 1.375410 mm | **1.178503 mm** |

C2 小幅移动后，两颗电容之间仍保留上述焊盘间距。局部验证要求每个 C1/C2 焊盘到所有顶层异网铜均 **≥0.25 mm**，实测全部满足。详细对象、网络及最短距离见 [local-clearance-verification.json](local-clearance-verification.json)。

## 验证与文件

- 保存关闭、从磁盘重新打开 A10 后，严格 PCB DRC 为 **0 项**，见 [pcb-drc.json](pcb-drc.json) 和 [reopened-check.json](reopened-check.json)。
- 独立铜几何检查确认 48 个网络连通、无异网短路；6 个 VIN 并联过孔双面接入，见 [copper-verification.json](copper-verification.json)。四路电机输出仍为底层约 1.5 mm 铜路，无换层过孔。
- 原理图网表与 PCB 的 32 个器件、143 个引脚网络逐脚核查；U4 既有 `ATTRMATCHCODE` 警告仍保留，见 [schematic-drc.json](schematic-drc.json)。
- 新 Gerber 中 C1/C2（含 C2 新位置）、U4/SW1/L1 焊盘、32 行器件坐标、19 组 BOM、模块孔距及完整原生工程 ZIP 的一致性见 [verification.json](verification.json)。

| 文件 | 用途 |
| --- | --- |
| [stm32PCB/stm32PCB.eprj3](stm32PCB/stm32PCB.eprj3) | 原生可编辑工程入口 |
| [stm32PCB-A10-project.zip](stm32PCB-A10-project.zip) | 完整工程压缩包 |
| [stm32PCB-A10-REVIEW-ONLY-Gerber.zip](stm32PCB-A10-REVIEW-ONLY-Gerber.zip) | 本修订制造数据，供重新评审 |
| [stm32PCB-A10-SMT-BOM.csv](stm32PCB-A10-SMT-BOM.csv) | 标准字段 UTF-8 BOM |
| [stm32PCB-A10-BOM.csv](stm32PCB-A10-BOM.csv) | EDA 原生 BOM |
| [stm32PCB-A10-PickAndPlace.csv](stm32PCB-A10-PickAndPlace.csv) | 包含 C2 新位置的器件坐标 |
| [PCB-top-silkscreen-A10.png](PCB-top-silkscreen-A10.png) | 顶视丝印预览 |
| [PCB-board-A10.png](PCB-board-A10.png) | 顶视双层铜路预览 |

复核入口：

```sh
python3 hardware/stm32PCB-A10/verify_design.py
# 下两项需要已有 Shapely 2.x 环境
python3 hardware/stm32PCB-A10/verify_copper.py
python3 hardware/stm32PCB-A10/verify_local_clearance.py
```

这是设计文件和制造数据的修订；尚未进行实物试插、完整三维装配、电源负载/温升或实车验证。原有 J13 槽孔与厂家推荐图的差异及 U4 属性匹配警告保留，详见 [原 BOM 核查](../../docs/BOM_5406283A_Y1-review.md)。没有下单或操作硬件；`hardware_tested=false`、`fabrication_release_approved=false`。A9 及更早制造文件不包含本次局部间距修改。
