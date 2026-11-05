# 2026-11-05 普通路口动作不因灰度失线立即停车

用户明确授权的新规则，取代历史普通转弯/离口“持续出口失效11/12/8立即停车”。在main当前未提交工作区接续，保留路径表、机械/速度/刹车配置、先前BRAKING修复及其他用户修改；未访问硬件、未烧录或复位。

## 行为对照与实现

| 场景 | 修改前 | 修改后 |
|---|---|---|
| 普通EXECUTING捕获出口后再次全丢线/歧义 | 独立400 ms/8 cm触发EXIT_LOST11、LINE_STOP_ROUTE | 继续原转角控制；灰度无效时冻结位置PID，不因失线退出/完成动作 |
| 普通转弯CLEARING失线 | 普通abnormal触发LINE_STOP_LOST | 仍朝已确认出口航向低速回正，受原离口阶段界限约束 |
| 普通STRAIGHT/非目标节点CLEARING失线 | route层400 ms/8 cm触发CLEAR_LINE_LOST12 | 使用原离口1500 ms/20 cm界限，超限转RECOVERING |
| 普通动作内一次图形保持耗尽1200 ms/12 cm | EXIT_RECAPTURE8立即停车 | 结束图形保持并回原角度/离口控制；used标志不清、不重开机会、不刷新阶段期限 |
| 目标STRAIGHT进入离口 | EXECUTING下一帧直接CLEARING，在CLEARING才确认出口 | EXECUTING须连续3帧居中窄出口；在该动作完成点锚定下一段，CLEARING仍须原去重 |
| 已确认节点阶段交接当帧旧abnormal已到期 | 普通失线检查先于route FSM，尚未交接就停车 | route FSM先处理真实节点证据；若成功进入普通受管阶段，预算交接，当帧不被旧abnormal停车 |
| 持续找不到出口/无法离口 | 可能先普通失线停车 | 阶段超限转RECOVERING，最后连续可靠线后持续失线1200 ms/12 cm仍停车 |

`src/line_follow.c:14`新增route_owns_gray_loss，仅让BRAKING、RECOVERING及普通非HEXAGON的EXECUTING/CLEARING管理灰度失线。`src/line_follow.c:459`保留普通失线到期结果；`src/line_follow.c:477`执行route_update，随后检查前后阶段所有权，避免进入或离开受管阶段的当帧被残留abnormal抢先停车。未确认CANDIDATE/FOLLOW仍不能靠孤立线重置到期结果；非路径模式保持原检查顺序和开关语义。

`src/route.c:649`移除普通转弯捕线后400 ms/8 cm的直接EXIT_LOST；普通CLEARING不再产生原失线12，但其阶段界限在`src/route.c:682`仍先于离口证据判定。`src/route.c:635`目标STRAIGHT补连续出口门禁，沿原居中/宽度/连续帧条件，在真实完成点保留总段距离。`src/route.c:281`图形保持耗尽按表项区分：HEXAGON原故障8不变，普通动作只回原阶段控制。

## 未放松的界限

- 出口确认仍必要：普通左右转、圆弧/自定义角沿原条件；自定义角仍需角度证据，目标STRAIGHT新增显式3帧出口门禁。未看到出口不能仅凭到角成功，也不能以失线补计或推进表项。非目标图形通过仍直接进入离口，不把它当作目标动作成功。
- 去重仍为连续窄线3帧/50 ms/2 cm，动作完成点与去重解锁分离，保留出口后距离。
- 当前配置普通90°动作是3000 ms，自定义角按比例延长，135°为4500 ms；STRAIGHT独立4500 ms，CLEARING为1500 ms/20 cm。本轮未把3000改为4500或延长其他阶段。
- 错向、过转及动作/离口超限仍按现有策略中止到RECOVERING，不将其认作成功。RECOVERING失线截止先于恢复，持续1200 ms/12 cm仍EXIT_LOST11、LINE_STOP_ROUTE停车，STOPPED仍需人工START。
- BRAKING实际轮速门禁、零目标、原1200 ms/10 cm/航向界限及速度合格无线中止到RECOVERING全部保留；先前entry-loss专项仍各2439项检查通过。
- 编码器计数跳变、堵转、确认反向、速度反馈/PID/输入异常、任务时序和俯仰锁存保护未修改；IMU故障仍按已授权规则连续降级编码器。非负轮目标、刹车接口及硬件确认开关、四项表值均保持。
- HEXAGON不在本轮范围：hexagon_update函数体与修改前逐字一致；ENTERING失线1200 ms/12 cm，离开ENTERING后400 ms/8 cm、一次图形恢复故障8仍可直接停车，公共离口也沿原灰度失线规则。完整HEXAGON矩阵继续通过。
- CANDIDATE未确认前仍直行且有普通失线保护；唯一相关交接变化是**真实确认节点的当帧**可转移旧预算，不能用未确认候选绕过停车。

## 测试与构建实际结果

1. 修改前独立运行executing、clearing、handoff用例，均退出134，见before-executing.log、before-clearing.log、before-handoff.log。保留两份修改前源文件，便于区分用户既有修改与本轮补丁。
2. 新专项使用C11、警告作为错误、ASan/UBSan：

```sh
sh test/run_maneuver_loss_tests.sh
```

四配置GRAY_INVERT=0/1 × MPU6050=0/1均退出0。编码器航向每配置9236检查，健康IMU航向输入每配置9240检查，见after.log。覆盖左右镜像、135°自定义角、捕线后再失线超过旧400 ms/8 cm仍有正向转角进度、出口证据不豁免、动作超限→RECOVERING→持续失线停车、转弯/直行离口失线、STRAIGHT连续出口门禁、到期候选进入EXECUTING/BRAKING的预算交接、计数与任务硬故障。使用合成编码器/IMU输入，不是物理轮速或真实IMU运行。

3. 更新旧回归中“普通动作失线必须立即停车”的断言，保留其他门禁。完整`sh test/run_host_tests.sh`最终退出0，85行PASS:（含重复配置），见host-tests.log；包括HEXAGON、入口2439项矩阵、两个极性、三个模式、IMU降级、反向/堵转/俯仰与时序等。新专项已接入完整入口。
4. 早期全套暴露旧断言冲突，记录host-initial.log、host-progress.log；一次静默输出执行在60秒idle watchdog处被中断，host-progress-2.log含Terminated 15，不算通过。最终改实时输出tee重跑全套成功；新定向夹具的自定义角捕线门槛、速度滤波预热已修正，未改生产阈值。
5. 普通`pio run`退出1：全局`~/.platformio/platforms.lock`权限失败，见pio-global.log。`PLATFORMIO_CORE_DIR="$PWD/.pio/hexagon-core" pio run`退出0，Flash42268 B、RAM10356 B，见pio-project-core.log。构建stderr有第三方HAL stm32f1xx_ll_utils.c的3项unused-parameter警告，未声称零警告；没有本项目编译错误。未运行上传命令。
6. README新增优先规则、同步两幅Mermaid；AGENTS追加用户规则。最终diff、表值、HEXAGON函数一致性与围栏/关键边检查见review-checks.log；未运行Mermaid渲染器。

## 验证边界

本轮没有现场日志，不认定历史实车根因，也不保证失线期间继续转向或低速离口一定能找到正确出口。取消普通动作内立即失线停车后，车辆可能在原阶段预算内继续运动；最终持续失线仍须停车。真实惯性、轮速跟随、IMU/编码器角度、抓地、出口形状和控制耗时均待实际授权后的实车验证。本轮没有改速度、路径表、刹车逻辑或硬保护来掩盖症状。
