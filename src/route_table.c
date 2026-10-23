#include "route.h"

/* 示例路径：实际使用前按赛道及编码器标定结果修改距离和速度。
 * 首段从启动位置起算；后续为用户测量的路口中心间距离，单位 cm。
 * 固件以动作触发点近似路口参考点，将转弯与离口的出口方向位移计入下一段；
 * 传感器到轮轴偏移和打滑仍须实测，不能认为参考点已精确对齐几何中心。
 * STOP 同样需要在距离窗口内确认路口；ROUTE_REPEAT_COUNT 控制总轮数，
 * 非最终轮的 STOP 标记直行/离口后回到首项。表内 40 cm/s 保留用户设置。 */
const RouteStep route[] = {
    {104.0f, 16.0f, LEFT, 40},
    {131.0f, 16.0f, LEFT, 40},
    {60.0f, 10.0f, LEFT, 40},
    {39.0f, 10.0f, RIGHT, 40},
    {70.0f, 10.0f, STOP, 40},
};
const size_t route_count = sizeof(route) / sizeof(route[0]);
