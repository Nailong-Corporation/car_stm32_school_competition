# 2026-11-04 总段距离 + 最多节点导航交付证据

> 本文保存上一阶段历史行为与验证。随后独立审查修正了动作完成锚点、离口减速包络和恢复PID/持续失线语义，当前规则与最终验证以[审查证据](../2026-11-04-navigation-review/README.md)为准；下文的离口完成原点、恢复阶段总时距门禁已被取代。

本轮在 `main` 接续已有未提交修改，未回退或重建工程。用户当前四项路径保留：LEFT/LEFT/LEFT/STOP，最多计数1/4/2/3，总段65/70/50/70 cm，turn_speed均50 cm/s，表速50/200/200/200。`src/route_table.c` 只同步字段注释、清理空白；未修改用户表值。不读取/连接硬件、不烧录、不操作电机。

## 当前行为

- 第7字段为 `segment_length_cm`，从START或上一目标动作离口确认点开始计算总段距离，包含中间图案节点；非目标节点通过不清零 `origin_cm` / `final_segment_origin_cm`。内部 `final_segment_*` 诊断名字沿用，语义已变为总段规划。
- 已确认节点数达到非零 `junction_count` 上限，或总段距离达到非零长度，任一满足后在灰度确认节点时执行表内动作。到距但只有普通单线时不转、不推进表项；孤立图案帧不能执行。计数0忽略计数；长度0未知，二者同0非法。STOP同样要求节点；循环配置 `ROUTE_REPEAT_COUNT=0` 保留，默认无限循环的末项STOP不是最终停车。
- 长度<=100 cm（含未知0）默认巡航60 cm/s；>100 cm用表速，并在原入弯减速开关/标定条件下运行速度平方距离包络。表中四项当前均短段，故默认巡航均为60，仍叠加启动、位置曲线、目标限幅、转弯/出口控制。60是目标基准，不是实测恒速。
- 多区域图案至少3目标通道可成为弱节点；仍须中心历史、连续帧、前进距离及离口去重。非目标候选和直行通过不额外按图案降速，保持当前规划速度并冻结位置PID、保持入口航向；原长段减速包络、启动与硬保护仍可降目标。真正目标节点候选会减速，入弯还须实测轮速合格。相同传感器序列仍可能把图案/路口混淆，不是语义识别。
- 距离耗尽不强转、不因旧搜口时间/距离停车；长段继续按入口速度搜口，超速记 `navigation_warning=BRAKE_DISTANCE(13)` 并继续减速。真实目标节点必须通过入口轮速门禁。

## 导航退路（控制器集成层）

`route_update()` 仍用 `ROUTE_FAILED` 报告失败；`line_follow` 在同周期调用 `route_recover_navigation()` 分类，软错误不会把电机命令中途清零为STOPPED。

| 原因 | 当前处理 |
| --- | --- |
| CONFIRM_TIMEOUT=4 | 候选中止，进入RECOVERING |
| ACTION_TIMEOUT=5 | 转弯/HEXAGON/离口阶段超时或超距，中止动作进入RECOVERING |
| TURN_LIMIT=6 / WRONG_WAY=7 | 中止转向，进入RECOVERING，不以错误角度宣告成功 |
| ENTRY_SPEED=10 | 原BRAKING仍先撤去正向驱动，1200 ms/10 cm/航向边界超限后转RECOVERING；不绕过速度门禁继续强转 |
| WINDOW=2 / SEARCH=9 | 兼容旧错误入口转RECOVERING；当前总段FOLLOW不主动产生旧窗口/搜口失败 |
| BRAKE_DISTANCE=13 | 仅更新警告并继续距离包络减速；当前不产生ROUTE_FAILED |

RECOVERING记录最近 `route.navigation_warning` 和累计 `navigation_recoveries`，接受START清零。目标上限16 cm/s，有连续可靠窄线时位置PID，否则冻结位置PID并保持进入恢复时的航向。连续单区域窄线至少3帧、50 ms且前进2 cm后返回FOLLOW；不推进路径索引、不清零计数和总段起点，不执行STOP回调。之后仍需新灰度节点取得动作资格。恢复不能保证重新回到正确赛道分支，尤其错向/过转后存在跟随错误线的风险。

恢复从进入阶段起独立计时/测距，1200 ms或12 cm任一先到且尚未建立可靠回线证据则EXIT_LOST停车；到期帧不能抢先恢复。证据中断会清空候选，不刷新阶段起点。恢复失败也可能发生在有零散窄线但始终未满足连续帧/时间/位移时，不能将其狭义描述为“传感器全零才停车”。

## 仍然停车的底层与出口条件

| 路径/来源 | 保留条件与结果 |
| --- | --- |
| 一般丢线/歧义 | 路径模式不受LOST_LINE_STOP_ENABLED关闭影响；最后稳定观测起400 ms或异常起点8 cm先到，LINE_STOP_LOST；孤立线不续时 |
| 路口出口持续失效 | EXIT_LOST=11、CLEAR_LINE_LOST=12：已捕获出口/直行通过缺乏可靠出口，原400 ms/8 cm门禁仍在；HEXAGON入环未捕线保留1200 ms/12 cm，捕线后400 ms/8 cm |
| 单次元件找线 | 原 COMPONENT_HOLD 单次1200 ms/12 cm未恢复中心线为EXIT_RECAPTURE=8；不属于普通阶段超时的软恢复豁免 |
| 新导航软恢复找线 | RECOVERING未在1200 ms/12 cm内满足连续窄线证据，EXIT_LOST=11。截止优先 |
| 路径/数据非法 | INVALID=1，包括非法表、越界索引、非有限里程/航向/轮速；非法制动规划参数也LINE_STOP_ROUTE；不尝试软恢复 |
| 里程与标定 | 尺寸/计数/方向标定不合法时拒绝START；里程更新失败、计数跳变/推算轮速超过650 cm/s或非有限结果为LINE_STOP_ENCODER |
| 速度内环 | 输入/周期/PID异常、目标>=3 cm/s而实测有符号轮速<0.5 cm/s持续350 ms堵转，或原反向确认门禁触发，LINE_STOP_ENCODER；低速短反向仍先双轮抑制，明显反向不豁免 |
| 调度/控制输入 | dt非有限、<=0或>15 ms，非法灰度分类或有效位置越界/非有限，PID失败为LINE_STOP_FAULT；main控制计算>=5 ms或任务超期仍停车 |
| 俯仰/初始化/看门狗 | 可用IMU的俯仰锁存、非IMU初始化失败保留停车；IWDG保留任务卡死复位退路。IMU数据失效本身仍锁定降级编码器，不能清除已锁存俯仰 |
| 人工/最终终点 | 人工按键STOP保留；有限循环最终STOP节点确认后LINE_STOP_ROUTE_COMPLETE、停车后一次回调 |

实际STOPPED不自动恢复；普通丢线/路径停车可显式START并重置路径，编码器/任务故障仍锁定需复位，俯仰走既有显式确认流程。零PWM/STBY低是待机滑行；不能把停止软件目标称为已验证机械制动。

原制动实现保留：`MOTOR_REVERSE_BRAKE_ENABLED=0` / `BRAKING_REVERSE_HARDWARE_CONFIRMED=0` 未改；满足原硬件确认与反馈门禁后才允许累计250 ms、60 PWM的反向力矩。`motor_short_brake()` 独立接口仍在，导航不自动调用。本轮未测制动力、硬件电流或实际停止距离。

## 最终验证

- `sh test/run_host_tests.sh > docs/evidence/2026-11-04-total-distance/host-tests-final.log 2>&1`：最终退出0，包含两种极性、三个控制模式、普通/自定义/HEXAGON导航及已有IMU降级/速度/时序/电机HAL回归，最后四行新增专项各1461项检查。PASS行是运行输出，不代表相同数量的独立实车用例。
- 新入口 `sh test/run_total_distance_tests.sh`，新增 `test/test_total_distance.c`，已接入完整脚本。四配置为两极性×IMU编译开关；直接合成分类观测及编码器反馈，不模拟真实电机和完整main/HAL按钮流程。覆盖距离先到、计数先到、0忽略计数、两者同0拒绝、STOP一次回调、100 cm边界、长段减速、图案候选与通过保速、总段不重建、七种软错误不停车/不跳项、四种硬路径错误停车、软恢复长失线停车、截止优先与任务锁定。完整套件含既有速度/反向/IMU硬保护回归。
- `pio run`：退出1，全局 `/Users/zhendong/.platformio/platforms.lock` 权限错误，见 `build.log`；没有请求改全局权限。使用已有项目内core目录：`PLATFORMIO_CORE_DIR="$PWD/.pio/hexagon-core" pio run > docs/evidence/2026-11-04-total-distance/build-local-core.log 2>&1` 最终退出0，唯一genericSTM32F103C8环境，Flash41760 B / RAM10348 B。
- 截止优先修正后已经重新跑完整主机套件和交叉构建；随后只收尾文档，不用旧成功结果覆盖新源码验证。
- `host-tests.log` 为首次中断前旧断言失败；`host-tests-resume.log` 为上一阶段续跑结果；`targeted-tests.log` 已在最终测试脚本清理修正后重跑，四配置各1461项检查通过；`host-tests-final.log` 保留完整套件的同版本源码结果。

本轮未烧录、未访问ST-Link/串口/硬件，未验证赛道识别率、目标轮速实际跟随、图案长度、短段入弯制动力或软恢复正确性。新RoutePlanner布局影响RAM诊断，后续读板须保留当时ELF，历史快照不能套用新布局。README中的旧日期现场记录是历史策略；当前精确行为以本说明及源码为准。

最终文档检查：Markdown围栏配对、当前字段名、用户四项路径值核对及 `git diff --check` 通过。未运行Mermaid渲染器，不声称图形渲染效果已验证。完整主机日志73行PASS为运行输出，包含重复配置，不能当成73项独立物理用例。

专项脚本使用工程内临时目录并在退出时删除编译器生成的二进制/dSYM，收尾发现的本轮遗留dSYM已清理；脚本清理变更后 `sh -n` 与四配置专项重跑通过，未改固件。
