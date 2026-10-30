# 2026-10-30 启动失败再次只读检查

用户再次报告启动失败。本轮只读运行状态，不检查烧录一致性，不复位、写目标、烧录或操作电机。已detach并退出OpenOCD。原始记录见gdb.txt。

- 调用栈main:126→Error_Handler；control_count=0，button_press_count=0，start_request_count=0，start_result=LINE_START_NONE。停在主循环之前，尚未进入PB10和灰度启动门禁。
- imu_init_failure_stage=2，探测0x68/0x69未取得合法WHO_AM_I；address_7bit=0x69仅表示最后尝试地址，不表示已探测到设备。
- imu.i2c.ErrorCode=32（0x20，HAL_I2C_ERROR_TIMEOUT）。未进入零偏校准。
- GPIOB IDR=0x00000d13：PB8/SCL=1，PB9/SDA=0。I2C1 SR2=0x00000002：BUSY=1。上一次独立读取也为SCL高、SDA低、BUSY=1。
- 当前直接阻塞是IMU通信超时和总线未回到空闲，不能由此确认是接线短路、缺上拉、模块异常还是从机未结束传输；仍需实际电气检查。
- 诊断中的encoder_control_enabled、imu_enabled等为0是初始化尚未走到赋值位置的默认值，不能解释为本次固件关闭这些功能。

现场下一步：先将MCU和IMU一起断电，检查SDA→PB9、SCL→PB8、3.3 V与共地，确认SDA未碰地且有3.3 V上拉，再一起重新上电并在校准阶段保持静止。仅按PB10不会退出已经进入的Error_Handler，也不会重新执行IMU初始化。若完整重新上电仍SDA低，需测量PB9/SDA电压并检查线缆/模块，不能通过取消启动门禁或修改路口参数解决。

本轮未改控制源码，因此未重新构建或运行主机测试。启动尚未恢复；前一轮减速修正的实车验证仍待完成。
