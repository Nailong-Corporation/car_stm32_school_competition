#include "route.h"
/* 字段顺序：路口数、转弯速度、动作、巡航速度、角度、圆弧开关、最后一段长度。
 * 路口数：上次动作离口后确认的第几个路口，实际数量需实车确认。
 * 转弯速度：入口实测轮速上限及转弯轮目标上限，单位 cm/s。
 * 动作：LEFT 左转、RIGHT 右转、STRAIGHT 直行、STOP 终点、CUSTOM_TURN 自定义转角。
 * 巡航速度：路段目标速度，单位 cm/s，不是 PWM。
 * 角度：仅 CUSTOM_TURN 使用，顺时针为正、逆时针为负，单位度。
 * 圆弧开关：true 为 ±90° 启用圆弧，false 使用普通转弯。
 * 最后一段长度：上一口离口确认点到目标口触发点的实测距离，单位 cm；0 禁用距离规划。
 * 旧路口中心间距离不能直接用于最后一段；启用距离规划前须确认制动标定。 */
const RouteStep route[] = {
    {1, 50.0f, LEFT, 50, 0.0f, false, 65.0f},
    {4, 50.0f, LEFT, 200, 0.0f, false, 70.0f},
    {2, 50.0f, LEFT, 200, 0.0f, false, 50.0f},
    {3, 50.0f, STOP, 200, 0.0f, false, 70.0f},
};
const size_t route_count = sizeof(route)/sizeof(route[0]);
