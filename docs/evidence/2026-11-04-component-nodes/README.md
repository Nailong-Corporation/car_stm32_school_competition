# 元器件节点实现与验证

基线 main：`4857be61d38434087a13cdd96ffa95b37f439098`。实现说明见 [component-nodes.md](../../component-nodes.md)。未烧录、未操作电机或调试硬件，所有掩码序列为合成输入；未证明实车识别率或通过速度。

## 软件验证

- `ASAN_OPTIONS=detect_leaks=0 sh test/run_host_tests.sh`：完整套件退出0，覆盖两种极性、IMU开关、三个控制模式及既有保护；日志为 `host-tests.log`。
- `python3 -m platformio run`：唯一生产环境 genericSTM32F103C8 构建成功，Flash 42428 B / RAM 11432 B；日志为 `platformio.log`。首次完整构建中厂商LL驱动存在未使用参数警告，本项目修改的源文件无新增编译警告。
- `git diff --check`：通过。
- 本环境 LeakSanitizer 不能读取受限 `/proc/.../task`，因此仅禁用泄漏探测；AddressSanitizer 与 UndefinedBehaviorSanitizer 保持启用。未因此调整仓库测试脚本的检查强度。

新增定向测试包含反向电源长短条顺序、电阻边框、多组部分触发、镜像、原始GPIO极性与多数滤波、无中心历史不计数且图形不更新位置PID、一次计数、固定穿越预算、连续出口、短相邻转弯、IMU航向纠正及当帧降级、编码器故障锁定。完整回归同时验证转弯后新节点交接使用出口航向，避免把旧入弯方向带入新穿越。既有直行无出口测试按新独立预算更新；转弯出口低速恢复用例仍保留。

## 旧代码失败与新代码通过

`repro-partial-node.c` 使用相同输入：连续4帧0x18中心线，随后连续4帧0x66，每帧前进0.06 cm。仅将 `route.c` 替换为上述main版本，其余分类/门禁相同：旧路口代码未计数，断言失败退出134；修改后同一断言通过退出0。见 `regression-before.log`、`regression-status.log`。这证明新增形状的漏计机制，不代表现场图形已经实测。

```sh
git show 4857be61d38434087a13cdd96ffa95b37f439098:src/route.c > /tmp/route-before.c
cc -std=c11 -Wall -Wextra -Werror -pedantic -Iinclude \
  docs/evidence/2026-11-04-component-nodes/repro-partial-node.c \
  /tmp/route-before.c src/gray_sensor.c src/braking_planner.c -lm -o /tmp/node-before
/tmp/node-before
# 将 /tmp/route-before.c 改为 src/route.c，同一用例退出0。
```

## 路线与发布边界

当前路线六项动作和数字保留，必须按「元器件 + 真实路口」统一序列重新标定。元器件单独作为目标项用STRAIGHT；三极管T分支按进出端配置。相同灰度形状无法保证区分元件与路口；若明确图形候选命中转弯/STOP目标则停车供核对，不盲目跳段。

本轮GitHub连接拒绝写入，远端new仍为 `40cacd707a8f43541f52639817c6c0af93155dd1`；本地new以main为基线完成改动。补丁需在获得写权限后应用。可在新的干净克隆中执行，避免覆盖已有工作：

```sh
git clone --branch main https://github.com/Nailong-Corporation/car_stm32_school_competition.git car-component-nodes
cd car-component-nodes
git switch -c new
test "$(git rev-parse HEAD)" = 4857be61d38434087a13cdd96ffa95b37f439098 || exit 1
git push origin new
# 上一步先将main快进同步至远端new；若远端另有新提交则拒绝，先检查而非强推。
git am /absolute/path/car_stm32_component_nodes.patch
sh test/run_host_tests.sh
pio run
git push origin new
```

本环境测试使用的ASAN_OPTIONS仅为环境适配，普通主机可直接运行原命令。若main已经变化，应重新审查补丁和回归，不跳过基线检查。
