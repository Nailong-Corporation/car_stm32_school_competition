# 导航独立审查续跑进度

main；用户四项路径表未修改，不访问硬件。

- 修改前三个缺陷及持续有线到期停车共四个独立入口均退出134，before-*.log保存断言。
- 已修：动作完成锚点与去重解锁分开；直行目标在连续居中窄出口3帧完成，普通/自定义转弯及HEXAGON在其原出口条件通过时完成；下一段预看沿用同锚点。非目标CLEARING和图案恢复保持总段包络并及时锁存到距。恢复PID进入时复位一次、可靠线延续历史；无可靠线复位；恢复期限自最后连续3帧可靠线起算，持续有线不因阶段年龄/距离停车。
- 第一轮专项四配置各435检查通过；完整套件首次被60秒静默超时中断，host-tests-interrupted.log保留；实时输出重跑完整套件退出0（host-tests.log）。PIO项目内core构建通过，Flash42128B/RAM10356B。
- 随后加强镜像/自定义角锚点、持续有线跨旧12cm阶段门槛及孤立帧不续失线预算回归；需再次最终重跑。余项：文档/证据同步、最终完整套件/构建/diff检查和清理本轮编译产物。

最终状态：实现和软件验证完成。补强后四配置审查专项各698检查、原专项各1461检查通过；最终完整主机套件退出0，host-tests.log。最终项目内core构建退出0，Flash42168B/RAM10356B，build-local-core.log。新增末轮STOP预看入口上限回归通过。README架构/状态Mermaid、AGENTS、navigation/hexagon和本目录README同步，旧总段证据标为历史。未跑Mermaid渲染器，无硬件验证；用户四项逐字段核对通过，临时二进制/dSYM已清理。

本轮写入：include/route.h、include/motion_config.h（失线预算注释）、src/route.c、src/line_follow.c、src/route_table.c（只同步动作完成基准注释）、test/test_navigation_review.c、test/run_total_distance_tests.sh、README.md、AGENTS.md、docs/navigation.md、docs/hexagon.md、docs/evidence/2026-11-04-total-distance/README.md（历史标记）、docs/evidence/2026-11-04-navigation-review/下README/progress及日志。其他既有未提交修改保留，不提交、不访问硬件。
