# 2026-10-30 路径距离 AUTO

距离列填写 `AUTO`，该项不校验或使用 tolerance，不设置距离窗口或提前距离减速。持续正常位置 PID 巡线；当前帧满足目标方向路口形状即执行，转弯在同周期使用不超过 min(表速,20 cm/s) 的规划值（普通转弯外轮 / 圆弧轴中点），不受提前减速开关控制。纯丢线骤变不触发 AUTO。数字距离模式保持原规则，当前启用路径表未自动替换距离。

AUTO 支持左右转、自定义角、STRAIGHT 与末项 STOP。最终 STOP 停车后分派一次回调；非最终轮保留闭合动作。下一项 AUTO 不进行距离越窗/短段提前交接，仍保留出口确认与正常离口去重；混合表项保留真实出口方向进度。

## 验证

- `auto-focused.log`：两种颜色极性 × 提前减速开关 0/1，共四组，各 305 次检查，AddressSanitizer/UndefinedBehaviorSanitizer 均通过。覆盖忽略 0/负值/大值/NaN/Inf 容差、长距离不越窗、普通线/丢线/反向分支不转、左右转与圆弧、CUSTOM_TURN、STRAIGHT、STOP/重复轮、检测后减速、正常离口、AUTO 与数字距离双向交接，以及数字窗口与非法参数校验。
- `host.log`：标准 `sh test/run_host_tests.sh` 中 AUTO 四组、原有八组速度开关与 HAL、启动门禁和 MPU 测试通过；随后在既有 `test_heading_scale.c:26` 距离比例断言失败，未宣称完整套件通过。
- `remaining-host.log`：单独继续执行该失败之后的测试（各二进制失败计数后继续），两种极性的历史导航均在 `test_spatial_short_s` / `test_navigation.c:2182` 交接断言失败，合计 2 个失败；其他后续 IMU/俯仰、仅巡线、PWM、丢线开关和地板细缝组合通过。上述两个既有失败在此前 [转弯速度开关记录](../2026-10-30-turn-speed-switches/README.md) 中已有同位置证据。
- `platformio.log`：`pio run` 成功，genericSTM32F103C8，RAM 3820/20480 B，Flash 37772/65536 B。
- `git diff --check` 通过。保留工作区原有 `JUNCTION_TURN_SPEED_CM_S=80` 修改。

未烧录、未操作电机、未实车验证；目标减速不代表实际制动距离，灰度形状可能误识别相似图形。传感器滤波仍在入口检测之前，动作超时、出口确认、编码器反馈、控制时序和当前配置的丢线保护继续生效。
