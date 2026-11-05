# 2026-11-04 独立审查三项修复

在main接续当前工作区，保留用户四项路径（LEFT/LEFT/LEFT/STOP，最多计数1/4/2/3，65/70/50/70 cm，表速50/200/200/200，turn_speed均50）。未访问硬件、未烧录、未操作电机。本说明优先于上一阶段 `../2026-11-04-total-distance/README.md` 中的完成点和RECOVERING阶段总时距规则。

## P1：动作完成点与去重解锁分开

`RoutePlanner.next_segment_origin_cm / next_segment_ms / next_segment_origin_valid` 保存下一段动作完成锚点。LEFT/RIGHT/CUSTOM_TURN在其原出口确认通过、进入CLEARING时建立锚点；HEXAGON仍需原配置净转角、新出口反侧分支和连续中心出口线证据，完成原子状态契约才建立锚点。STRAIGHT目标在CLEARING首次连续3帧合格居中窄出口时定义动作完成；不会把刚进入图案直行通过的时刻视为已完成。

后续50 ms/2 cm只解锁节点计数。`complete_clear()`把已锁存锚点交给 `origin_cm / final_segment_origin_cm / segment_ms`，不再用解锁当帧的d重建基准；已沿出口前进的行程保留到下一段。中间非目标节点仍不改变总段基准。

`route_planning_index/origin/entry_speed_limit`让目标CLEARING立即预看下一项长度、速度、入口上限与同一完成锚点；索引仍等去重完成才推进。下一段已经到距时即时置search_pending，切段也保持该资格；距离仍不能省略下一节点灰度确认。单行STOP循环预看下一轮时，不把当前闭合动作的lap_boundary套到下一轮最终STOP入口上限。

## P1：离口和图案恢复持续同一距离包络

普通FOLLOW/CANDIDATE、非目标CLEARING及其单次图案恢复均持续使用当前段总距离和速度平方减速包络；目标CLEARING在完成锚点建立后使用下一段预看包络。目标转弯离口仍另受原16 cm/s或更低表内上限限制。

长段制动不再在非目标CLEARING中关闭，也不在图案恢复中重回巡航。`braking_origin_cm`只在规划基准真正改变时切换，非目标通过沿用同一基准；跨总段长度时，CLEARING立即建立search_pending并请求入口低速。这里的“保速”是保持当前规划包络，包络可以下降，不代表恒速或实测轮速立即到位。真实入口轮速门禁、原反向力矩确认门禁和短刹接口保持。

## P2与用户优先规则：恢复PID历史与真实持续失线

RECOVERING进入阶段复位位置PID一次；连续可靠单区域窄线使用同一个PID，保留微分/积分历史。无可靠线或尚未连续确认时复位并保持进入恢复时的航向；重新确认线后重建历史，不把图案当作位置误差。

1200 ms / 12 cm只作为RECOVERING持续失线预算，从最近连续至少3帧可靠窄线起计算。持续可靠线会刷新失线基准，即使没有移动到2 cm解锁点、阶段已超过1200 ms或阶段累计超过12 cm，也不会仅因阶段年龄/总行程停车。孤立一两帧、宽线、多区域、采样中断不刷新该基准；失线截止检查先于迟到的恢复帧。

返回FOLLOW仍需连续窄线3帧、50 ms且前进2 cm，不推进路径项、不清零计数和当前总段起点，也不把原错误动作认作成功。持续可靠线但车轮真正不动的情况仍由独立速度堵转保护处理，不能用解除导航阶段时限掩盖硬件反馈故障。

## 导航退路与仍停车原因

- 候选确认超时4、动作/离口时距超限5、过转6、错向7、入口降速失败10转RECOVERING。旧WINDOW2/SEARCH9兼容入口也走软恢复；当前总段FOLLOW不产生旧搜口限界。距离耗尽超速13仅警告并继续减速，无灰度节点不盲转。
- RECOVERING只有从最近连续可靠线起持续失线1200 ms/12 cm超限，才以EXIT_LOST11停车；不再按进入恢复阶段以来的固定总时距停车。
- 原一般丢线/歧义400 ms/8 cm、已捕获出口失效11和直行无可靠出口12保留；HEXAGON入环未捕线保留1200 ms/12 cm，捕线后仍400 ms/8 cm；原单次COMPONENT_HOLD未满足中心出口证据的1200 ms/12 cm仍为EXIT_RECAPTURE8。这些是各自可靠出口证据的门禁，未取消。
- 非法路径/配置/里程/航向/轮速/制动规划仍停车；未标定拒绝START。编码器计数跳变/速度合理性超限、堵转（目标>=3且有符号实测<0.5 cm/s持续350 ms）、确认反向、速度PID/输入故障仍停车并保留原锁定。
- dt非法或>15 ms、非法灰度分类/位置、控制PID异常、主任务控制计算>=5 ms或调度超期、可用IMU俯仰锁存、非IMU初始化故障仍停车；IWDG保留卡死复位退路。IMU失效仍锁定降级编码器，不清除俯仰锁存。
- 人工STOP及有限循环最终STOP停车保留。STOPPED不自动续跑，START门禁和硬故障锁定不变。当前ROUTE_REPEAT_COUNT=0仍无限循环。

`MOTOR_REVERSE_BRAKE_ENABLED=0`、`BRAKING_REVERSE_HARDWARE_CONFIRMED=0`未改；原有界反向力矩和独立 `motor_short_brake()` 接口保留，导航不启用未经验证的反向/短刹。STBY低待机不能称为实测短刹。

## 实际验证与证据

1. 修改前新增入口anchor/braking/pid/reliable分别退出134，`before-*.log`保留首个断言，证明当时工作区可复现四个机制；并非实车根因证据。随后调整了STRAIGHT/HEXAGON夹具的采样终点以及LEFT相对航向符号，并扩展镜像/自定义角回归，最终测试源码以 `test/test_navigation_review.c` 为准。
2. 新审查专项已接入 `test/run_total_distance_tests.sh`，两极性×IMU开关四配置，ASan/UBSan、-Wall/-Wextra/-Werror/-pedantic，各698项检查通过；原总段专项四配置各1461项仍通过，见 `targeted-tests.log`。覆盖LEFT/RIGHT/CUSTOM_TURN/STRAIGHT/HEXAGON完成锚点、去重后下一节点到距、单项STOP末轮预看、减速区非目标CLEARING及图案恢复、跨阈值限速、目标离口预看、第二帧微分有效、持续可靠线跨旧阶段时距、孤立帧不续失线预算。
3. 最终 `sh test/run_host_tests.sh 2>&1 | tee docs/evidence/2026-11-04-navigation-review/host-tests.log` 退出0；工具执行环境启用pipefail，覆盖完整旧套件和上述新专项，含两极性、三个控制模式、IMU降级、普通/自定义/HEXAGON导航、编码器/速度/任务/俯仰硬保护及HAL模拟回归。最初静默重定向运行被工具60秒无输出看门狗终止，保留 `host-tests-interrupted.log`，不算通过。
4. 使用已有项目内core：`PLATFORMIO_CORE_DIR="$PWD/.pio/hexagon-core" pio run 2>&1 | tee docs/evidence/2026-11-04-navigation-review/build-local-core.log` 最终退出0，genericSTM32F103C8，Flash42168 B / RAM10356 B。沿用上一阶段全局锁权限受限的环境，本轮未重新尝试默认core或改全局权限。最终代码定稿后重跑了完整主机套件和该构建；随后仅收尾文档。

局限：新测试通过合成分类灰度/编码器反馈驱动软件，不证明实际刹车、出口捕获、轮速响应、失线恢复到正确分支或赛道成绩。持续可靠线超1200 ms但无位移的回归直接调用RoutePlanner，以隔离导航条件；生产集成的堵转保护仍由原完整速度套件验证。原HEXAGON拓扑/角度契约未放宽，但本轮新增锚点夹具从已有EXITING状态验证交接，完整入环/新出口分支由原HEXAGON套件覆盖。新字段改变RAM布局，后续硬件诊断须保留读取时ELF。

收尾核对：用户四项表值逐字段保持；README架构/状态Mermaid和Markdown围栏检查、`git diff --check`通过。未运行Mermaid渲染器，不声称图形渲染效果已验证。`intermediate-*.log`保留夹具调整前的中间尝试，不作为最终通过证据；最终结果以targeted-tests.log/host-tests.log/build-local-core.log为准。本轮生成的临时回归二进制及dSYM已清理。
