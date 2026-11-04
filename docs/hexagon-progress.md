# HEXAGON 续跑进度

2026-11-03：源码、定向主机测试、完整回归、交叉构建及文档交付完成。接续时先核对了 git status/diff、AGENTS.md 和已有实现，没有重做已完成部分。现有 src/route_table.c、PCB及其他用户差异保留；示例未应用到实际路径表。未访问、烧录或操作硬件。

## 已完成

- `include/route.h`：HEXAGON 枚举与独立 HexagonNavigation 子状态；RouteStep 字段顺序不变。
- `include/motion_config.h`：独立入环/总动作/出环/失线初始限界。
- `src/route.c`：ENTERING → FOLLOWING → EXIT_CANDIDATE → EXITING → DONE，再走公共 CLEARING；计数仅确认入环口，环内分支不追加路径计数。角度从 begin_action（必要入口降速之后）开始，包含入环、净累计不取模、左负右正。到角前分支不可挪用；到角后的中心走廊开启新出口资格。活动图形恢复期间的分支不能绕过期限。
- `src/line_follow.c`：稳定 FOLLOWING 的 `use_line` 由连续窄线建立，使用灰度位置 PID 与双轮速度环，没有持续固定转弯偏置；入环/异常分支低速引导，出口候选保持切线航向。IMU当帧降级不清零路径角。
- `test/test_hexagon.c`、`test/run_host_tests.sh`：两极性×IMU关/开ASan/UBSan；回归到角持续单线不出环、镜像方向、出口证据中断、旧分支不挪用、图形恢复期限、时距/错向/过转/无效反馈与任务锁定，并调用原导航回归。修正了START重置中心历史后测试夹具缺少前置中心帧的问题，未放宽生产门禁。
- `README.md`：总体架构Mermaid增加HEXAGON、正常PID/异常保持/独立停车数据流；运行状态Mermaid增加外层EXECUTING中的HEXAGON子状态。普通动作路径保持。
- `docs/navigation.md`：增加当前HEXAGON入口，历史记录保留。
- `docs/hexagon.md`：字段对照、可复制完整C数组示例、状态图、精确限界、参考差异及拓扑/实车边界。
- `AGENTS.md`：适度追加当前语义、初值及验证记录。

## 已执行验证及准确命令

1. `sh test/run_host_tests.sh > docs/hexagon-host-tests.log 2>&1`：最终退出0。日志共73行包含PASS，其中HEXAGON每次还打印其调用的通用导航结果；四种组合9141/9145/9141/9145项检查，包含通用导航检查数，不能当成独立HEXAGON用例数。完整套件含两极性、三个控制模式、普通LEFT/RIGHT/CUSTOM/STOP及循环回归。
2. 首次 `pio run > docs/hexagon-build.log 2>&1` 因不能写全局 `/Users/zhendong/.platformio/platforms.lock` 失败。创建项目内 `.pio/hexagon-core`，其 packages/platforms 符号链接复用本机已安装工具链，再执行 `PLATFORMIO_CORE_DIR="$PWD/.pio/hexagon-core" pio run > docs/hexagon-build.log 2>&1`：退出0，Flash41248 B、RAM10340 B。未变更固件模式、路径表或PlatformIO环境。当前构建日志保留最终成功结果，首次失败原因在本文说明。
3. 文档收尾只修改README、导航、HEXAGON说明、AGENTS及本进度；没有改动固件/测试，故未重复完整主机套件或交叉构建。
4. 从 `docs/hexagon.md` 提取完整C示例，以 `cc -std=c11 -Wall -Wextra -Werror -pedantic -Iinclude -x c -fsyntax-only -` 通过stdin编译检查；解析去注释的RouteStep字段并逐项核对顺序通过。文档代码围栏配对检查通过；未运行Mermaid渲染器，不声称渲染效果已验证。
5. 最终 `git diff --check` 退出0；无空白错误。

## 风险审阅与实车余项

- 本次收尾审阅未发现需要更改固件的新缺陷；代码依据是 `hexagon_update` 的稳定窄线/use_line、控制器HEXAGON的pid_update分支、begin_action角度基准及里程层(l-r)/track的右正符号。
- 出口要求完整配置角度后中心走廊加新反侧分支，再有中心窄线3帧、100 ms、0.5 cm。右侧顶点接线只有满足切线直行延续和绕行反侧分支才适用，尚未确认实际入口朝向/连接角；300°仅示例。不能声称识别六边形几何或必能区别矩形图形。
- 入口3000 ms/30 cm、入口未捕线1200 ms/12 cm、总动作30000 ms/400 cm、出口3000 ms/30 cm；捕线后400 ms/8 cm，图形单次恢复1200 ms/12 cm；公共离口1500 ms/20 cm。错向为归一化净角小于-0.35 rad，过转为配置最小角度再120°，不是每个反向增量都立即停车。
- 正常动作最高轮目标16 cm/s（并受表内turn_speed约束）；真实轮速响应、距离与编码器航向比例、赛道识别、控制周期耗时均需实际测量。新子状态改变RAM布局，后续诊断须保存读取时ELF。
- 参考指定GitHub实际读取 `User/CircleHandler.c`，借鉴分阶段、方向分支及出环连续证据；保留本工程8路GPIO/HAL及角度+出口联合门禁。参考目录JSON和源文本分别留在 `docs/hexagon-reference-tree.json`、`docs/hexagon-reference-CircleHandler.c.txt`。

软件交付无待补项；硬件验证仍待后续授权与实测，不自动续跑、不烧录、不操作电机。
