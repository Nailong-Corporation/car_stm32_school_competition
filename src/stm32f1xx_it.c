#include "main.h"

/**
 * @brief 处理系统滴答中断，仅递增 HAL 毫秒时基，不在中断内执行控制算法。
 */
void SysTick_Handler(void) { HAL_IncTick(); }
/**
 * @brief 不可屏蔽中断入口，转入统一异常处理以请求电机停车。
 */
void NMI_Handler(void) { Error_Handler(); }
/**
 * @brief 硬故障中断入口，转入统一异常处理以请求电机停车。
 */
void HardFault_Handler(void) { Error_Handler(); }
/**
 * @brief 内存管理故障入口，转入统一异常处理以请求电机停车。
 */
void MemManage_Handler(void) { Error_Handler(); }
/**
 * @brief 总线访问故障入口，转入统一异常处理以请求电机停车。
 */
void BusFault_Handler(void) { Error_Handler(); }
/**
 * @brief 指令或处理器使用故障入口，转入统一异常处理以请求电机停车。
 */
void UsageFault_Handler(void) { Error_Handler(); }
