#include "route.h"

/* 示例路径：实际使用前按赛道及编码器标定结果修改距离和速度。
 * 每段距离从启动位置或上一动作完成、离开路口后的位置起算，单位为 cm。
 * STOP 同样需要在距离窗口内确认路口；本文件仅定义数据，没有函数。 */
const RouteStep route[] = {
    {30.0f, 5.0f, LEFT, 12},
    {20.0f, 4.0f, STOP, 8},
};
const size_t route_count = sizeof(route) / sizeof(route[0]);
