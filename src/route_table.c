#include "route.h"

/* 示例路径：实际使用前按赛道及编码器标定结果修改距离和速度。
 * 每段距离从启动位置或上一动作完成、离开路口后的位置起算，单位为 cm。
 * STOP 同样需要在距离窗口内确认路口；本文件仅定义数据，没有函数。 */
const RouteStep route[] = {
    {104.0f, 8.0f, LEFT, 25},
    {131.0f, 8.0f, LEFT, 25},
    {60.0f, 5.0f, LEFT, 25},
    {39.0f, 5.0f, LEFT, 25},
    {70.0f, 5.0f, STOP, 25},
};
const size_t route_count = sizeof(route) / sizeof(route[0]);
