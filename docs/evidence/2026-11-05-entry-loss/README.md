# 2026-11-05 首段冲出、不转向后停车：源码调查与修复

在 main 当前未提交工作区上接续，完整读取 AGENTS.md 与 README；保留既有修改。没有现场日志，没有访问 ST-Link、开发板、电机、Flash 或硬件寄存器，没有烧录、复位或发送运动请求。以下说明源码机制及主机验证，不认定它们就是这次实车根因。

## 首段实际配置与时序

`src/route_table.c:11` 四项保持为：

| 项 | 最多节点数 | 动作 | 总段 cm | 表内巡航 cm/s | turn_speed cm/s |
|---|---:|---|---:|---:|---:|
| 0 | 1 | LEFT | 65 | 50 | 50 |
| 1 | 4 | LEFT | 70 | 200 | 50 |
| 2 | 2 | LEFT | 50 | 200 | 50 |
| 3 | 3 | STOP | 70 | 200 | 50 |

角度均0，圆弧开关均false。ROUTE_REPEAT_COUNT仍为0；生产测试夹具为核验最终STOP而设置单轮，未改生产开关。首段65 cm属于短段，`route_cruise_speed()`返回默认60 cm/s，并非表内50。当前长段距离减速包络只用于>100 cm，所以不能把首段65 cm假定为提前制动曲线。这里未改任何速度或规划阈值。

`route_target_ready()`（src/route.c:449）采用最多计数或总段距离OR；首项计数1允许第一次连续确认的节点执行，无须等满65 cm；无线、距离先到或单帧突变不能触发动作。中心历史、侧别连续、多帧、位移与去重仍须满足。目标LEFT同侧强分支可用2帧/0.05 cm，其他强/弱证据保留原门槛。

真实三帧多数滤波在稳定输入后延迟一帧反映新分支；新`test_filtered_entry()`以真实极性转换和滤波器连接控制器，输入0x18→0x1f，在三个5 ms控制周期依次获得FOLLOW、CANDIDATE、BRAKING。这个合成例子只证明输入与状态顺序，不是实车识别延迟的测量。

目标节点确认后计数一次，实测任一轮超过有效入口上限min(50,16)=16 cm/s，则进入BRAKING（src/route.c:593）。该阶段请求双轮零目标，保持原速度内环、零输出/待机逻辑；实际轮速未降到门槛前不发起转向。当前未启用反向力矩或短刹，降低目标不能证明物理轮速已经下降。因此“确认了节点但暂时完全不转向”是入口门禁允许的过程，不能单凭症状断言漏检。

## 两个已证明的软件问题

1. **候选异常标志未交接。** 目标CANDIDATE不进入普通位置PID，会设置`f->abnormal`。旧BRAKING分支未清除它，后续src/line_follow.c:448的普通400 ms/8 cm截止在route_update之前停车，可能先于BRAKING原1200 ms/10 cm/航向界限，跳过已经授权的导航软恢复。旧源码下`test_budget()`在仍超速、尚未越过入口界限时断言失败（退出134，before-budget.log）。它能解释“仍在零目标制动而未转向，随后普通丢线停车”这一软件路径；不证明现场经过它。
2. **速度已合格但无线时仍停留在零目标等待。** 旧src/route.c BRAKING只在`entry_speed_ready && line_mask!=0`时推进。滑行穿过分支后，若低速/静止且本帧无线，零目标无法主动重获线，只能等入口超限；与上一个标志问题组合还可能直接普通丢线停车。旧源码下`test_loss_ready()`断言应进入现有RECOVERING失败（退出134，before-loss.log）。该问题是恢复时机缺陷，不是允许无线转弯的理由。

## 最小修复及保护边界

- src/line_follow.c:585：已经确认节点、实际进入BRAKING后清除旧普通异常标志；入口使用原1200 ms/10 cm/航向限界，仍请求零目标。确认前的普通丢线截止位置未动，到期帧不能靠新节点迟到豁免。
- src/route.c:604：实际入口速度合格但本帧全无线，以原ENTRY_SPEED=10中止本次动作，交给已有route_recover_navigation；有线且速度合格才正常accept_target。入口限界检查仍在accept之前。
- RECOVERING保持原16 cm/s上限、可靠窄线PID/无线航向保持，连续窄线3帧/50 ms/2 cm只返回FOLLOW；不宣称动作完成、不推进索引、不清零计数或总段基准，之后需新灰度节点确认。
- 恢复持续失线原1200 ms/12 cm先到仍以EXIT_LOST=11停车；孤立线不续预算，截止优先。未确认节点的普通400 ms/8 cm、出口丢线、图形恢复及阶段边界保持。本修复是确认节点后的有界预算交接，绝非路径模式关闭丢线停车。
- 编码器跳变、堵转、确认反向、输入/PID异常、任务时序及俯仰锁存等保护未修改。刹车源文件、接口、反向硬件确认及生产速度/机械参数均未修改。无新增RAM结构字段。

## 回归与实际结果

定向入口命令：

```sh
sh docs/evidence/2026-11-05-entry-loss/run_regression.sh
# 可追加 budget / loss / bounds / guards / filtered 单独定位
sh test/run_host_tests.sh
pio run
PLATFORMIO_CORE_DIR="$PWD/.pio/hexagon-core" pio run
```

- 两个修改前失败：before-budget.log、before-loss.log，均退出134；失败断言针对本轮两个目标行为，生产四项表参与。
- 修改后第一次完整定向测试仍失败，保存为after-initial-fixture.log。原恢复期限夹具持续输入零编码器进度；fixture-diagnosis.log读取**停车快照**确认reason=4（LINE_STOP_ENCODER）、speed_fault=4（STALL），进入恢复460 ms后停车。实时speed.fault已被停车复位，不能拿其0值否认快照故障。
- 修正期限夹具：采用每5 ms左右各2计数，在合成每计数0.01 cm标定下为4 cm/s，1200 ms内未到12 cm且不会触发堵转。额外独立静止回归必须触发STALL，并验证新START不能解除锁定。没有放宽任何堵转参数。
- 最终四配置IMU=0/1 × GRAY_INVERT=0/1，各2439项检查通过，ASan/UBSan、C11、警告作为错误，退出0；见after.log。覆盖真实滤波/极性链、实际速度门禁、零目标超速阶段、预算交接、速度合格无线不转、恢复成功保留进度、恢复1200 ms和12 cm分别停车、截止优先、静止堵转锁定、普通未确认失线、编码器计数跳变。
- IMU矩阵为编译开关组合；新控制器夹具通过line_follow_update使用现有编码器航向/失效降级入口，不冒称健康IMU实物模拟。完整既有套件保留健康IMU输入、当帧降级、三个控制模式、反向及任务故障等用例。
- 最终`sh test/run_host_tests.sh`退出0，日志81行以PASS:开头（包括重复配置，不代表81个独立套件），见host-tests.log；新增定向入口已接入完整脚本。增加真实滤波用例后重跑完整套件。
- 普通`pio run`退出1，`/Users/zhendong/.platformio/platforms.lock`出现PermissionError，见pio-global.log。使用既有项目内core后退出0，Flash42172 B、RAM10356 B，见pio-project-core.log。源码在成功构建后未再修改，后续仅补充主机测试与文档。
- README总体架构及运行状态两幅Mermaid已同步入口轮速门禁、BRAKING异常交接、速度合格无线转RECOVERING；围栏/关键边及diff检查通过，未运行Mermaid渲染器。最终静态检查见review-checks.log。

## 仍未确定的现场原因

没有本次原始输入、导航trace、轮速、阶段及停车快照，不能判断现场属于未形成候选、灰度分支过短/侧别变化、入口轮速超限滑行、BRAKING无线、恢复丢线，还是独立编码器/调度故障；也不能确认当前源码就是板上版本。新轮径和编码器比例、实际轮速/惯性/抓地与传感器几何均未在本轮验证。修复不会保证已冲出后必然找回正确线，持续无线仍须停车；也没有凭空修改速度、表值或放松硬保护来消除症状。

后续有实际授权和数据时，区分route_before/after、junctions_seen、candidate_reject、实际左右轮速、stop_snapshot.reason/route_failure/speed_fault即可确定此次现场走过的路径。本轮没有读取或操作任何硬件。
