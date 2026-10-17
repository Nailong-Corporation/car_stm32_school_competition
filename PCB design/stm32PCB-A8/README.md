# STM32 载板 A8：两组模块排距改为 15.24 mm

> **历史修订：** 2026-10-17 用户指出 TB6612FNG 上下两排反向，已在 [A9](../stm32PCB-A9/README.md) 交换 J8/J9 并重新布线、验证和导出。A8 的 DRC 结果不证明实物针序匹配，A8 制造包不包含此次方向修正。

2026-10-17，按用户指定，将 STM32F103C8T6 开发板和 TB6612FNG 模块插座的两排孔中心距统一为 **15.24 mm（600 mil）**。基于 A7 另存本修订，旧版保留。原生工程入口为 [stm32PCB/stm32PCB.eprj3](stm32PCB/stm32PCB.eprj3)。

| 模块 | 插座 | 两排孔中心距 | 单排相邻孔中心距 |
| --- | --- | --- | --- |
| STM32F103C8T6 开发板 | J6 / J7，两排各 20 孔 | **15.24000 mm** | 2.54 mm |
| TB6612FNG 模块 | J8 / J9，两排各 8 孔 | **15.24000 mm** | 2.54 mm |

这里的排距指两排孔的**中心线距离**，不是排母塑料外壳的边缘距离。15.24 mm 是本次用户指定的尺寸，不据此推断所有同芯片模块都使用该排距。

## 修改范围

- 保留 J7、J9 位置，将 J6、J8 向对应排内移约 0.762 mm；再按原生坐标对齐对应孔，消除编辑器焊盘读数舍入与制造坐标之间的小于 1 微米的排距误差。
- 同步相连走线端点，重布受影响的 PC13、PC14、PC15、VBAT_EXT、NRST、BTN、PWMA 信号，修整相邻地线并重新生成双面地铜。设计规则、主要电源铜路宽度和底层 1.5 mm 电机输出铜路保留。
- STM32 上排 20 个引脚丝印随插座移动；两组排距丝印、板尾 A8 版本字样和原理图尺寸说明同步更新，见 [原理图文字修改记录](schematic-note-updates.json)。
- 保留 A7 的 32 个器件、143 个焊盘网络、48 个网络、供应商料号及模块引脚顺序；STM32 顶视仍为 **USB 朝左、SWD 朝右**。A7 的 U4 焊盘、基础库矮按键和 C5/C6/L1 间隙修复保留，依据见 [A7 说明](../stm32PCB-A7/README.md)。

## 验证

- 保存关闭 A8 后从磁盘重新打开，严格 PCB DRC 为 **0 项**。器件位置、旋转、封装及料号、143 个焊盘、全部丝印、走线、过孔和地铜与保存前快照匹配，见 [reopened-check.json](reopened-check.json)和 [pcb-drc.json](pcb-drc.json)。
- 独立铜几何检查确认 48 个网络连通、无异网短路，6 个 VIN 并联过孔均双面接入，见 [copper-verification.json](copper-verification.json)。
- 从新导出的 `Drill_PTH_Through.DRL` 逐对核对 **20 对 STM32 孔和 8 对 TB6612 孔**，全部排距均为 **15.24000 mm**；另核对各插座孔、2.54 mm 单排针距、100 × 90 mm 板框与四角安装孔。
- 原理图与 PCB 的 143 个引脚网络一致；原理图仍有 A7 已有的一条 U4 `ATTRMATCHCODE` 警告，见 [schematic-drc.json](schematic-drc.json)，未关闭或豁免该警告。
- 复核 Gerber 中 U4/SW1/L1 焊盘、全部 32 行坐标和 19 组 BOM；工程 ZIP 与磁盘原生工程逐文件相同，见 [verification.json](verification.json)。

## 文件

| 文件 | 用途 |
| --- | --- |
| [stm32PCB-A8-project.zip](stm32PCB-A8-project.zip) | 完整可编辑工程，解压后打开 `stm32PCB/stm32PCB.eprj3` |
| [stm32PCB-A8-REVIEW-ONLY-Gerber.zip](stm32PCB-A8-REVIEW-ONLY-Gerber.zip) | **15.24 mm 排距**的制造数据，供重新评审 |
| [stm32PCB-A8-SMT-BOM.csv](stm32PCB-A8-SMT-BOM.csv) | UTF-8 BOM，标准字段；供应商封装名称沿用 A7 核查版本 |
| [stm32PCB-A8-BOM.csv](stm32PCB-A8-BOM.csv) | EDA 原生 BOM，UTF-16、制表符分隔 |
| [stm32PCB-A8-PickAndPlace.csv](stm32PCB-A8-PickAndPlace.csv) | A8 原生坐标，UTF-16、制表符分隔，mm 单位 |
| [PCB-top-silkscreen-A8.png](PCB-top-silkscreen-A8.png) | 顶视丝印预览 |
| [PCB-board-A8.png](PCB-board-A8.png) | 顶视双层铜路预览 |
| [stm32PCB-A8.netlist.json](stm32PCB-A8.netlist.json) / [pcb-state.json](pcb-state.json) | 原理图网表及 PCB 核查快照 |

复核入口：

```sh
python3 hardware/stm32PCB-A8/verify_design.py
# 使用已有 Shapely 2.x 环境
python3 hardware/stm32PCB-A8/verify_copper.py
```

本次完成设计文件及制造数据修改，尚未实物试插、全板三维装配、贴装样板、电源负载/温升或实车验证。J13 槽孔与厂家推荐图的既有差异及 U4 属性匹配警告仍保留，见 [原 BOM 核查](../../docs/BOM_5406283A_Y1-review.md)及 [A7 说明](../stm32PCB-A7/README.md)。没有重新下单或操作硬件；`hardware_tested=false`、`fabrication_release_approved=false`。A7 及更早的 Gerber 不包含本次排距修改。
