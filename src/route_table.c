#include "route.h"

/* 示例路径：实际使用前按赛道及编码器标定结果修改距离和速度。
 * 首段从启动位置起算；后续为用户测量的路口中心间距离，单位 cm。
 * 固件以动作触发点近似路口参考点，将转弯与离口的出口方向位移计入下一段；
 * 传感器到轮轴偏移和打滑仍须实测，不能认为参考点已精确对齐几何中心。
 * STOP 同样需要在距离窗口内确认路口；ROUTE_REPEAT_COUNT 控制总轮数，
 * 非最终轮的 STOP 标记按 ROUTE_REPEAT_ACTION（当前 LEFT）转向/离口后回到首项。表内 60 cm/s 保留用户设置，须不超过 WHEEL_TARGET_MAX_CM_S。 */
/* 第五列 angle_deg 仅 CUSTOM_TURN 使用；例如：
 * {75.0f, 10.0f, CUSTOM_TURN, 60, 45.0f},  顺时针 45°
 * {75.0f, 10.0f, CUSTOM_TURN, 60, -135.0f}, 逆时针 135°
 * 有效范围 [-360, 0) 或 (0, 360]；直行用 STRAIGHT。 */
const RouteStep route[] = {
    /*** 周博洋的赛道
    {104.0f, 16.0f, LEFT, 50, 0.0f},
    {131.0f, 16.0f, LEFT, 50, 0.0f},
    {60.0f, 10.0f, LEFT, 50, 0.0f},
    {39.0f, 10.0f, RIGHT, 50, 0.0f},
    {70.0f, 10.0f, STOP, 50, 0.0f},
    */
    /* 赛道 2
    {75.0f, 10.0f, LEFT, 60, 0.0f},
    {219.0f, 16.0f, LEFT, 60, 0.0f},
    {75.0f, 10.0f, LEFT, 60, 0.0f},
    {219.0f, 16.0f, STOP, 60, 0.0f},
    */
    // 赛道 3
    //{76.0f, 5.0f, CUSTOM_TURN, 60, 90.0f},
    //{82.0f, 5.0f, CUSTOM_TURN, 60, -90.0f},
    //{10.0f, 5.0f, CUSTOM_TURN, 50, -60.0f},
    {55.0f, 5.0f, CUSTOM_TURN, 60, 120.0f},
    {55.0f, 5.0f, CUSTOM_TURN, 60, -60.0f},
    {52.0f, 8.0f, LEFT, 60, 0.0f},
    {82.0f, 8.0f, LEFT, 60, 0.0f},
    {80.0f, 8.0f, LEFT, 60, 0.0f},
    {220.0f, 15.0f, STOP, 60, 0.0f},
};
const size_t route_count = sizeof(route) / sizeof(route[0]);
