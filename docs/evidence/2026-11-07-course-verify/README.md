# 2026-11-07 整圈完成 + 27 印刷元件覆盖：独立验证工具与基线

本目录只做**验证**：不改仿真器 `src/`/`tests/`，不改固件 `src/`/`include/`，不烧录、不操作硬件。
目的是给“两条参考路径是否真的跑完整圈、是否每个印刷元件都经过”一个可重复、可复核的判据，
而不是替 Lead 调参。

## 1. 文件

| 文件 | 说明 |
| --- | --- |
| [`verify_course.mjs`](verify_course.mjs) | 验证脚本：spawn 仿真器回放 → 解析 CSV → 判定整圈/元件/分步/失败现场 |
| [`baseline-20261106T221228-report.txt`](baseline-20261106T221228-report.txt) | 2026-11-06 22:12:28 (+0800) 基线完整报告（逐方案、逐步、逐元件、失败尾部 20 帧；由该次回放的同一份 CSV 快照用 `--reuse` 重新生成，数值未变） |
| [`baseline-20261106T221228-fingerprint.txt`](baseline-20261106T221228-fingerprint.txt) | 同一时刻的 git 状态与源文件 sha256，用来锚定“这份基线对应哪份代码” |
| [`baseline-20261106T221228-scheme3-extra.log`](baseline-20261106T221228-scheme3-extra.log) | 附带检查：方案 3（外框测试圈）同一时刻也失败，见 §5 |

回放产生的 `scheme{1,2}.csv`（每次约 1.9 MB / 3.3 MB）与 `report.json` 不放进仓库，默认写在
`/tmp/course-verify/`；用同一条命令即可随时重生。

## 2. 运行

在固件仓库根目录（`/Users/zhendong/Documents/PlatformIO/Projects/car_stm32`）执行：

```sh
node docs/evidence/2026-11-07-course-verify/verify_course.mjs
# 指定输出目录（建议每轮一个新目录，便于 diff）
node docs/evidence/2026-11-07-course-verify/verify_course.mjs --out /tmp/cv-$(date +%H%M%S)
# 只跑方案 1
node docs/evidence/2026-11-07-course-verify/verify_course.mjs --schemes 1
# 离线重解析已有 CSV（不重新回放）
node docs/evidence/2026-11-07-course-verify/verify_course.mjs --reuse --out /tmp/cv-2212
```

参数：`--sim DIR`（默认 `/Users/zhendong/Documents/stm32car_simulator`，也可用环境变量
`STM32_SIM_DIR`）、`--route-table FILE`（默认本仓库 `src/route_table.c`）、`--out DIR`、
`--schemes 1,2`、`--pass-cm 6`、`--timeout-ms 300000`、`--reuse`、`--allow-fail`（归档已知失败
基线时强制退出 0）、`--quiet`。

退出码：`0` 两个方案都整圈且 27 个元件全部覆盖；`1` 有方案未完成/未覆盖全；`2` 回放进程本身
失败（例如路径表方案号不存在、TS 编译错误）；`3` 脚本用法或环境错误。

耗时：两个方案合计约 1.4 s（仿真回放本身约 0.45 s/0.49 s），适合反复迭代。

## 3. 判定口径（重要，勿误读）

**数据来源（全部只读）**

- 行驶轨迹：直接复用仿真器自带的 `tests/replay.mjs`（`cwd` = 仿真器目录，读固件真实
  `src/route_table.c`），逐周期 CSV 列 `ms,index,state,x,y,h,mask,pos,...,failure,line`。
  本脚本故意**不重新实现**仿真物理，避免“验证器和被验证对象一起错”。
- 元件清单：用建 `replay.mjs` 同样的 vite SSR 加载 `src/track/map_data.ts`，直接取
  `CIRCUIT_COMPONENTS`（19 电阻 + 7 三极管 + 1 电池 = 27）与 `START_POINT`。不复制、不硬编码坐标。

**整圈完成**：CSV 末行 `state == ROUTE_FINISHED` 且 `failure == 0`。同时报告最终索引
（对照路径表步数）、最终位姿、最终位姿到起点 P(0,110) 的距离。仅“index 走到最后一步”不算完成，
仅“回到 P 附近”也不算完成。

**元件经过**：对每个元件中心求**车轴轨迹折线**的最短距离（逐个采样点 + 逐个线段），
`<= --pass-cm`（默认 6 cm）记为经过；同时给出连续进入半径的**次数**（≥2 记为 `PASS xN`，
用于识别“重复通过”，不是失败）。另外独立计算**传感器杆中心轨迹**（车轴 + 24 cm × 航向）的
最短距离与次数——那才是 8 路灰度真正扫过的点，作为第二判据列出。缺失项附最短距离、最近一次
接近的时刻/索引/状态，便于判断是“没走到那段”还是“擦边而过”。

**分步表**：按 CSV 的 `index` 列切段，给出每步进入/退出时刻、时长、位姿、退出时的
`junctions_seen`、`navigation_recoveries`、`navigation_warning`、退出状态与动作类型；并检测
**跳号**（index 一次跳多步）与**回访**（同一步骤被再次进入），两者都是路径状态机的异常信号。

**失败现场**：未完成时打印最后 20 个控制周期（ms/index/state/count/x/y/mask/pos/line/failure）。

**步内诊断表**：每一步额外给出 ① **枢轴起点**（航向相对该步进入航向偏离 > 0.15 rad 的第一个周期，
`-` 表示整步没转）及其位姿、② **最后一个 `mask≠0` 的周期**（灰度最后一次看到线）及其位姿/掩码/状态。
“EXIT_LOST” 的两类典型现场——转角没确认（无枢轴、在导线末端变全黑）与确认点提前太多
（枢轴点离几何转角很远、转完不在导线上）——都直接体现在这两列里。

## 4. 基线结果（2026-11-06T22:12:28+08:00，当前未提交工作树）

指纹（详见 fingerprint 文件）：`car_stm32` HEAD `58b34fc`，工作树有未提交改动；
仿真器 HEAD `9b48611`，`src/sim/{config,gray_sensor,line_follow,physics,route,route_parser,types}.ts`、
`src/track/{map_data,track_renderer}.ts`、`tests/replay.mjs` 均为 `M`（Lead 正在改）。
关键 sha256：`line_follow.ts c7ced3bc…`、`route.ts 8b59b865…`、`physics.ts e43638aa…`、
`map_data.ts abba215d…`、`tests/replay.mjs 25e76353…`、`src/route_table.c 756d8688…`。
回放前后对上述文件做了 sha256 对比，**基线运行期间无文件变化**；两次独立回放结果完全一致（确定性）。
另：22:14 之后 Lead 又改了 `config.ts`、`line_follow.ts`、`route.ts`（基线之后 sha256 已变），
所以下表只是 **22:12:28 那一刻**的锚点；要看当前状态请重跑本脚本，别引用这份数字当现状。

| 方案 | 结果 | 时长 | 最终索引 | failure | 元件覆盖（车轴 / 传感器） |
| --- | --- | --- | --- | --- | --- |
| 1 走虚线（30 步） | **未完成** | 28.48 s | 5 / 29 | `ROUTE_FAILURE_EXIT_LOST`(11) | **8 / 27**（6） |
| 2 不走虚线（28 步） | **未完成** | 49.52 s | 9 / 27 | `ROUTE_FAILURE_EXIT_LOST`(11) | **10 / 27**（8） |

方案 1 已通过步骤 0–4（P→左上角→(55,0)→(55,140)→(245,140) Q6→(245,170)），
**卡在第 5 步“y=170 西行至 (95,170)，过 Q11”**：23.865 s 进入，28.480 s 失败，共 4.6 s。
失败前最后姿态 `(88.05, 169.49)`、航向 3.155 rad（向西），即车轴已向西越过 x=95 约 7 cm，
仍在 y=170 线上；此后 `mask=0` 持续、`ROUTE_RECOVERING` 用掉 1 次恢复预算后 `EXIT_LOST`。
两条关键事实（都可在报告的 step diagnostics 里直接读到）：
① 该步**从未发生枢轴**（全步航向偏离进入值从未超过 0.15 rad），即 (95,170) 转角没有被确认；
② **`junctions_seen` 到停止时只有 1（Q11），目标是 2**；最后一个 `mask≠0` 的采样在
`(117.5, 169.9)`（mask=12，仍在 ROUTE_CLEARING），此后全黑直至停车。
（解释性推测，未作为结论：24 cm 前伸使传感器杆中心在车轴距转角约 24 cm 时已越过 x=95，
该处中心落在导线边缘之外，属于“90° 转角前伸盲区”这一类问题。）

方案 2 已通过步骤 0–8（含 Q6 左转北上、y=40 西行、x=165 南下、y=80 东行回 (245,80)），
**卡在第 9 步“y=40 西行过 x=165/R23 至 (95,40) 右转北上”**：42.235 s 进入，49.520 s 失败。
`junctions_seen` 已达到目标 3 并进入 `ROUTE_EXECUTING`，但**枢轴发生在 `(121.1, 41.0)`
（46.000 s），比几何转角 (95,40) 提前约 26 cm**；最后一次 `mask≠0` 在 `(113.2, 22.1)`
（mask=1，已在 EXECUTING 中向北偏离导线），最终 `(111.52, -2.37)`（**越过顶边 y=0 约 2.4 cm**）
的纯 `mask=0` 状态下 2 次恢复后 `EXIT_LOST`。也就是说这一次“节点已确认、也转了”，
但转完不在任何导线上；成因（元件图样被计为节点 / 前伸导致确认点在转角之前 / 转后无回找）
本脚本不做定论。

覆盖明细（车轴 6 cm 判据）：

- 方案 1 覆盖 8：`bat1, r1, r2, r3, r5, q4, q6, q11`；缺 19：`r8 r9 r10 r12 r14 r15 r17 r19 r20
  r21 r23 r24 r25 r26 r27 q22 q16 q18 q13`。
- 方案 2 覆盖 10：`bat1, r1, r2, r3, r5, r23, r24, r25, q4, q6`；缺 17：`r8 r9 r10 r12 r14 r15
  r17 r19 r20 r21 r26 r27 q22 q16 q18 q13 q11`。其中 `r25` 出现 `PASS x2`（两次进入 6 cm），
  与官方图 2“重复通过 7/13/14/18/19/24”的标注同属预期重复，不判失败。

## 5. 附带发现：方案 3（外框测试圈）也在同一时刻失败

不在默认检查范围内，用 `--schemes 3` 复现（日志见 `baseline-…-scheme3-extra.log`）：

- 14.63 s、索引 1/4、`EXIT_LOST`，最终 `(382.09, 0.20)`，`junctions_seen=7`（目标 11）。
  车沿顶边东行**越过右边框 x=375 约 7 cm**，即 (375,0) 直角转角同样未被确认。

与方案 1 的 (95,170)、方案 2 的 (95,40) 合并看，当前工作树的一致现象是
**“线在转角处结束/转弯、但转角未被确认为节点”**。历史上同构的外框闭合圈曾在
`docs/evidence/2026-11-06-new-track/replay-lap.csv`（10394 周期 / 51.97 s，当年是表内方案一，
5 步 105/370/215/370/105，与现在的方案 3 相同）跑出 `ROUTE_FINISHED`，因此当前现象更像是
本轮改逻辑的中间态，值得 Lead 优先确认是否预期。本脚本只报事实，未改任何源码。

## 6. 局限（不要把这些当成已验证的结论）

1. **几何近似**：元件覆盖用“车轴中心到元件中心 6 cm”判定，没有建模 8 路灰度对元件 footprint 的
   实际读数，也没有建模元件笔画（空心电阻、V 口）；6 cm 是覆盖判据，不是电气“读到”的证明。
   传感器杆中心距离只作参考列，未被当作判据。
2. **坐标来自仿真地图**：元件位置/尺寸是仿真标定假设（见
   `docs/evidence/2026-11-06-new-track/README.md`：电阻 8.0×3.8 cm 为图样比例的 55 %，
   实物尺寸未实测）。地图一改，基线就变；脚本每次都从 `map_data.ts` 现读，故会跟着更新。
3. **元件编号有两个体系**：`CIRCUIT_COMPONENTS.elementNumber` 是图样标号（`R25` → 25，
   0…27 中缺 7 号），**不是** `docs/evidence/2026-11-06-official-route/README.md` 里
   “元件编号即首次通过顺序”的官方顺序号（那里 R25 是元件 7）。报告同时打印 `id`/`label`/
   `elementNumber`，覆盖判据与编号无关。
4. **单次仿真、无随机性**：回放是确定性的（已对比两次结果一致），所以“通过”只代表该仿真配置下
   通过，不代表实车；也未覆盖 IMU 降级、打滑、电池电压等。
5. **整圈判据依赖 `ROUTE_FINISHED`**：若路径表最后一个 STOP 是循环末项，需确认出现的是
   `ROUTE_FINISHED` 而不是回到索引 0；本脚本会把“索引回退/回访”单独列出。
6. 不检查固件 C 与 TS 是否同步：本工具只验证仿真器行为；C 侧仍需 `sh test/run_host_tests.sh`。

## 7. 给 Lead 的用法建议

- 每轮改完 `src/sim/*` 后跑一次并保存新目录，例如
  `node docs/evidence/2026-11-07-course-verify/verify_course.mjs --out /tmp/cv-<hhmm> --allow-fail`，
  再 `diff <(grep '^scheme' /tmp/cv-2212/report.txt) <(grep '^scheme' /tmp/cv-<hhmm>/report.txt)`
  看两个方案各自推进到第几步、覆盖数变化。
- 想判断“整圈真的完成”时，除了 `overall: PASS`，再看三项：`ROUTE_FINISHED/yes`、
  `final route index N of N`、`components covered 27/27`（传感器列仅参考）。
- 出现 `skipped indices` 或 `re-entered indices` 时先查路径状态机，别先调参数。

## 8. 本轮未做的事

- 未修改 `/Users/zhendong/Documents/stm32car_simulator/src`、`tests` 下任何已有文件（只读 + spawn）。
- 未修改本仓库 `src/`、`include/`、`test/`。
- 未烧录、未连接或操作硬件；未运行固件主机测试（不在本任务范围）。
