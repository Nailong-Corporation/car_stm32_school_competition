# 2026-11-04 续跑进度

分支 main；保留用户四项路径表（65/70/50/70 cm），不访问硬件。

已接续：总段距离/计数上限任一先到，在真实灰度节点确认后执行；0 忽略计数；短段默认60，长段距离包络；图案非目标节点保持规划速度；导航软恢复保留索引、计数和总里程。此前通过日志 host-tests-resume.log 早于最后两处候选保速修改，不作为最终验证。

当前阶段：补 test_total_distance.c 专项与独立运行入口，再运行完整主机套件、pio run，同步 README/Mermaid/AGENTS/证据。最终结果以本文件后续追加和 README.md 为准。

硬件闭环/任务/标定非法、持续无可靠出口线仍有停车路径；动作超时/转角错误/入口降速超限转低速有界恢复，不冒充动作成功、不推进表项；距离耗尽超速保留减速和警告。精确分支清单待最终源码核对后写证据。

阶段结果：专项四配置各1455检查通过；完整套件退出0（host-tests-final.log）。普通 pio run 因全局 platforms.lock 权限失败（build.log），沿用既有项目内 PLATFORMIO_CORE_DIR="$PWD/.pio/hexagon-core" pio run 成功（build-local-core.log，Flash41756B/RAM10348B）。随后审查发现软恢复到期帧可先接受线证据，已调整为截止优先并新增时间/距离边界回归；因此这些成功结果需由最终重跑更新。接下来文档同步和最终验证。

最终状态：软件余项已完成。最终完整套件退出0，host-tests-final.log包含截止优先新增边界后四配置各1461检查（73行PASS含重复配置）。最终项目内core构建退出0，Flash41760B/RAM10348B，build-local-core.log。README总架构/状态Mermaid、AGENTS、docs/navigation.md、docs/hexagon.md和本目录README已同步，后者含准确软退路和仍停车清单。普通pio run全局锁失败仍保留build.log，不宣称普通命令通过。无硬件访问/烧录/电机操作。未跑Mermaid渲染器。

写入/接续文件：include/motion_config.h、include/route.h、src/route.c、src/line_follow.c、src/route_table.c（仅注释/空白，保留四项用户值）、test/test_junction_evidence.c、test/test_navigation.c、test/test_turn_handoff.c、test/test_turn_settling.c、test/test_turn_speed_switches.c、test/test_total_distance.c、test/run_total_distance_tests.sh、test/run_host_tests.sh、README.md、AGENTS.md、docs/navigation.md、docs/hexagon.md及本证据目录。前两轮已有修改不回退、不创建提交。

收尾：专项临时目录改为统一退出清理，移除本轮生成的dSYM；sh -n与专项四配置重跑通过，targeted-tests.log更新为最终各1461项。固件未再修改；最终git diff --check通过，路径四项原值核对通过。
