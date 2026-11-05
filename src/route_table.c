#include "route.h"
/* 字段顺序：最多路口数、转弯速度、动作、长段巡航速度、角度、圆弧开关、总段长度。
 * 总段从 START / 上次动作完成点测量，中间图案节点通过不重建里程。
 * 计数或距离任一先到，在灰度确认节点后执行；计数 0 忽略计数，长度 0 未知。
 * 长度 <=100 cm（含未知）默认巡航 60 cm/s；>100 cm 使用表速和距离减速包络。
 * turn_speed 为入口及动作上限，转弯另受 16 cm/s 限速与实测入口轮速门禁。
 * CUSTOM_TURN / HEXAGON 角度右正左负；圆弧开关只用于普通 ±90°。
 * 制动、速度反馈与可靠出口线门禁独立保留；配置不等于实测标定。 */
const RouteStep route[] = {
    /* 赛道 1 */
    {1, 50.0f, LEFT, 50, 0.0f, false, 65.0f},
    {4, 50.0f, LEFT, 200, 0.0f, false, 219.0f},
    {2, 50.0f, LEFT, 200, 0.0f, false, 70.0f},
    {3, 50.0f, STOP, 200, 0.0f, false, 219.0f},


};
const size_t route_count = sizeof(route)/sizeof(route[0]);
