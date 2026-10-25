#include "route.h"

/* 示例路径：实际使用前按赛道及编码器标定结果修改距离和速度。
 * 首段从启动位置起算；后续为用户测量的路口中心间距离，单位 cm。
 * 以入口灰度触发点近似路口中心；转弯/离口累计灰度中心出口方向净位移。
 * 已计入前置 16 cm 的旋转位移，不以轮程/探头弧长代替中心间距离；打滑仍须实测。
 * R=10 的完整直角结束时探头已在出口约 26 cm；下面 20 cm 短段可能在回线前越过，
 * 保留用户路径表和真实进度，不能保证此短段路口可捕获。
 * STOP 同样需要在距离窗口内确认路口；ROUTE_REPEAT_COUNT 控制总轮数，
 * 非最终轮的 STOP 标记按 ROUTE_REPEAT_ACTION（当前 LEFT）转向/离口后回到首项。表内 60 cm/s 保留用户设置，须不超过 WHEEL_TARGET_MAX_CM_S。 */
/* 第六列 corner_arc_enabled：true 启用本项 90° 圆弧；false 使用原转弯。
 * STOP 非最终轮映射为左右转时也使用本项开关，最终轮仍正常停车。
 * STRAIGHT / 非 ±90° CUSTOM_TURN 不走圆弧，无论开关值。
 * 第五列 angle_deg 仅 CUSTOM_TURN 使用；例如：
 * {75.0f, 10.0f, CUSTOM_TURN, 60, 45.0f, false},  顺时针 45°
 * {75.0f, 10.0f, CUSTOM_TURN, 60, -135.0f, false}, 逆时针 135°
 * 有效范围 [-360, 0) 或 (0, 360]；直行用 STRAIGHT。 */
const RouteStep route[] = {
    /*** 周博洋的赛道
    {104.0f, 16.0f, LEFT, 60, 0.0f, true},
    {131.0f, 16.0f, LEFT, 60, 0.0f, true},
    {58.0f, 12.0f, LEFT, 60, 0.0f, true},
    {55.0f, 12.0f, CUSTOM_TURN, 50, 90.0f, true},
    {20.0f, 10.0f, LEFT, 50, 0.0f, true},
    {60.0f, 16.0f, STOP, 60, 0.0f, true},
    */
    /* 赛道 2
    {75.0f, 10.0f, LEFT, 60, 0.0f, true},
    {219.0f, 16.0f, LEFT, 60, 0.0f, true},
    {75.0f, 10.0f, LEFT, 60, 0.0f, true},
    {219.0f, 16.0f, STOP, 60, 0.0f, true},
    */
    /* 赛道 3 */
    //{76.0f, 5.0f, CUSTOM_TURN, 60, 90.0f, true},
    //{82.0f, 5.0f, CUSTOM_TURN, 60, -90.0f, true},
    //{10.0f, 5.0f, CUSTOM_TURN, 50, -60.0f, true},
    {55.0f, 5.0f, CUSTOM_TURN, 60, 120.0f, true},
    {55.0f, 5.0f, CUSTOM_TURN, 60, -60.0f, true},
    {52.0f, 8.0f, LEFT, 60, 0.0f, true},
    {82.0f, 8.0f, LEFT, 60, 0.0f, true},
    //{80.0f, 8.0f, LEFT, 60, 0.0f, true},
    {220.0f, 15.0f, STOP, 60, 0.0f, true},

};
const size_t route_count = sizeof(route) / sizeof(route[0]);
