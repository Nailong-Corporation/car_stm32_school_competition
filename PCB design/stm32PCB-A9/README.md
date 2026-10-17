# STM32 载板 A9：修正 TB6612FNG 上下两排插接方向

> **历史修订：** 后续按用户截图调整 C1/C2 周边间距的修订为 [A10](../stm32PCB-A10/README.md)。A9 保留 TB6612 上下排方向修正，但不包含 A10 的 C2 小幅位移、电源支路和地铜调整。

2026-10-17，按用户指出的 TB6612FNG 上下两排反向问题，基于 A8 另存为 A9，**交换 J8 与 J9 的实际位置及相关走线、丝印**。两排孔中心距仍为 **15.24 mm**，单排相邻孔中心距仍为 **2.54 mm**。A8 原文件保留，A8 制造包不包含这次方向修正。

## 修正后的插座方向

以下以载板**顶面朝向观察者、`FRONT ^ GRAY 3.3V` 在上、`REAR` 在下**为准，每排从左向右排列：

| 位置 | 插座 | 从左到右的 8 个引脚 |
| --- | --- | --- |
| 上排，靠近 STM32 | **J9** | **PWMA、AIN2、AIN1、STBY、BIN1、BIN2、PWMB、GND** |
| 下排，靠近板尾 | **J8** | **VM、VCC、GND、AO1、AO2、BO2、BO1、GND** |

`VM` 接板载 5 V 电机电源，`VCC` 接 3.3 V 逻辑电源。两排编号同向，左端 `J9-1 PWMA` 对应下方 `J8-1 VM`，右端为两排的第 8 脚 GND。翻到载板底面时，左右位置随观察方式镜像，装配判断须以这一顶视定义和模块针脚标注为准。

本次按用户反馈交换两排，保留各排内部的左右针序；没有取得新的 TB6612 模块照片，也没有进行实物试插，因此不宣称已完成模块机械配合验证。逐针坐标、网络和修改依据见 [方向核查记录](tb6612-orientation-verification.json)。STM32 的 J6/J7 方向与 A8 一致，顶视 USB 朝左、SWD 朝右。

## 走线与验证

- 重布 PWMA/PWMB、AIN1/AIN2、BIN1/BIN2、STBY、AO1/AO2/BO1/BO2，以及移动后的 VM/VCC 支路；修整两段冲突旧地线并重新生成双面地铜。
- 四路电机输出保持**底层铜路、至少约 1.5 mm 宽、不使用换层过孔**；主要电源铜路及 6 个 VIN 并联过孔、2 个 MCU 5 V 并联过孔保留。
- 从磁盘关闭重开 A9，严格 PCB DRC 为 **0 项**。器件、焊盘、走线、过孔、地铜和丝印与导出前快照在原生坐标精度内匹配，见 [重开记录](reopened-check.json)及 [PCB DRC](pcb-drc.json)。编辑器初次检查曾出现连接提示，重新打开后不再出现；最终结果采用重开工程的检查。
- 独立铜几何检查确认 **48 个网络连通、无异网短路**，VIN 的 6 个并联过孔双面接入，见 [铜检查](copper-verification.json)。
- 重新读取原理图网表，32 个器件 / 143 个引脚网络与 PCB 逐脚一致。原理图仍有既有的 **U4 `ATTRMATCHCODE` 警告**，没有关闭或豁免，见 [原理图 DRC](schematic-drc.json)。
- 核对新 Gerber 中 20 对 STM32 孔、8 对 TB6612 孔均为 **15.24000 mm** 排距；同时核对 U4/SW1/L1 焊盘、32 行器件坐标、19 组 BOM 和原生工程 ZIP，见 [验证报告](verification.json)。核查脚本新增 TB6612 上下位置、每排针序及对应丝印的断言。

## 文件

| 文件 | 用途 |
| --- | --- |
| [stm32PCB/stm32PCB.eprj3](stm32PCB/stm32PCB.eprj3) | 原生可编辑工程入口 |
| [stm32PCB-A9-project.zip](stm32PCB-A9-project.zip) | 完整工程压缩包 |
| [stm32PCB-A9-REVIEW-ONLY-Gerber.zip](stm32PCB-A9-REVIEW-ONLY-Gerber.zip) | 包含本次方向修正的制造数据，供评审 |
| [stm32PCB-A9-SMT-BOM.csv](stm32PCB-A9-SMT-BOM.csv) | 标准字段 UTF-8 BOM，料号与 A8 相同 |
| [stm32PCB-A9-BOM.csv](stm32PCB-A9-BOM.csv) | EDA 原生 BOM |
| [stm32PCB-A9-PickAndPlace.csv](stm32PCB-A9-PickAndPlace.csv) | 新器件坐标，包含 J8/J9 位置交换 |
| [PCB-top-silkscreen-A9.png](PCB-top-silkscreen-A9.png) | 顶视丝印预览 |
| [PCB-board-A9.png](PCB-board-A9.png) | 顶视双层铜路预览 |

复核入口：

```sh
python3 hardware/stm32PCB-A9/verify_design.py
# 需要已有 Shapely 2.x 环境
python3 hardware/stm32PCB-A9/verify_copper.py
```

本次为 PCB 接口修订，无固件、MCU 引脚或 PID 策略变更。尚未实物试插、全板三维装配、电源负载/温升或实车验证；J13 槽孔与厂家推荐图的既有差异、U4 属性匹配警告仍保留，详见 [原 BOM 核查](../../docs/BOM_5406283A_Y1-review.md)与 [A7 说明](../stm32PCB-A7/README.md)。没有下单或操作硬件；`hardware_tested=false`、`fabrication_release_approved=false`。
