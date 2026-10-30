# 2026-10-30 路径笔误修正后再次启动失败

用户仍无法启动。本轮只读调试，不检查烧录一致性。gdb.txt保存调用栈与诊断，registers.txt保存随后独立寄存器读取；已detach并退出OpenOCD，未复位目标、烧录、写目标或操作电机。

## 当前失败

control_count=0、start_result=LINE_START_NONE、button/start_request计数0，调用栈main:126→Error_Handler。MPU初始化阶段2、最后尝试地址0x69、I2C ErrorCode=32（0x20超时）。路径尚未初始化，不能据诊断默认ROUTE_FOLLOW或valid=false认定路径再次非法；也尚未发生倾斜启动拦截。

补读GPIOB IDR=0x0f53，PB8/SCL=1、PB9/SDA=1；I2C1 SR2=0，BUSY=0。失败时超时与稍后总线空闲同时存在，不能断言一直短路或一直拉低；也不能仅凭当前空闲确定原故障是MCU外设内部状态还是模块/供电/接触的暂态。

## 修改

- src/mpu6050.c：在一轮双地址探测失败、且尚有下一轮时，HAL_I2C_DeInit、RCC强制复位/释放I2C1、HAL_I2C_Init，等待原20 ms后继续。最多4次恢复、5轮双地址WHO_AM_I探测；永久无响应仍阶段2失败，DeInit/Init失败为阶段1。只有WHO_AM_I和后续配置/新样本/校准全部通过才ready。
- 初始化读写使用MPU6050_INIT_I2C_TIMEOUT_MS=25；ready后的运行期仍MPU6050_I2C_TIMEOUT_MS=2，不在运行任务里重试或外设复位。初始化始终在电机待机、IWDG启动前进行；未改按键、有效线、路径、倾斜或控制时序门禁。
- imu.init_recovery_attempts可通过GDB读取，记录实际恢复次数。README同步。

ST RM0008的I2C复位描述提供异常状态恢复的参考：https://www.st.com/resource/en/reference_manual/cd00171190.pdf 。这里不把超时确认为硅片勘误，也不宣称单纯外设复位可修复所有BUSY状态、外部SDA拉低或物理连接问题。主机测试不能验证电气恢复。

## 验证

imu-test.log：ASan/UBSan通过。新增需重初始化后才能WHO_AM_I成功的场景、恢复Init失败安全退出、持久无响应耗尽原次数上限、初始化25 ms与运行期2 ms参数分离。原两地址探测、配置/新样本失败、校准中位数、启动门禁、静止/90°积分用例保留通过。

start-test.log：实际生产路径与配置START专项通过，包括在线接受、无线路径/IMU故障拒绝与编码器正反馈保护。

platformio.log：pio run成功，RAM13044/20480 B，Flash38708/65536 B。git diff --check通过。未重跑无关导航全套，历史heading_scale/spatial_short_s失败未宣称已修复。

新恢复逻辑尚未烧录/实车验证。当前板仍停在Error_Handler，不会因源码修改而自行重试。首次重新上电校准仍须静止；若新固件全部恢复轮次仍失败，需实际检查PB8/PB9、3.3 V供电、共地和上拉，不能跳过IMU要求强行开车。
