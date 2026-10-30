# 2026-10-29 在线无法启动

只读ST-Link检查，未复位、烧录、写入目标或操作电机；已detach并关闭OpenOCD。

读取前ELF为before.elf，转换before.bin共37,056 B，与flash.bin前缀逐字节一致。原始结果见[gdb.txt](gdb.txt)。

- 调用栈：main:126 → Error_Handler，尚未进入控制循环。
- imu_init_failure_stage=2，最后探测地址0x69，I2C ErrorCode=32（HAL_I2C_ERROR_TIMEOUT）。两个地址未能读到匹配的WHO_AM_I；无法从这一次快照确定是模块启动时序、接线、供电或持续总线故障。
- control_count=0、start_request_count=0、start_result=NONE；灰度字段为未采样初值，不能用它断言线上/无线。
- 校准尚未运行（bias/raw/span=0），本次与上一轮新增零偏均值/波动门禁无关。

修复上电一次失败后立即锁死的脆弱入口：启动先等待100 ms，再最多5轮探测0x68/0x69；每失败轮间隔20 ms。成功后继续既有配置和静止校准，持续失败仍保持停止。延迟仅在电机待机、控制循环和看门狗启动前执行，不进入5 ms控制任务。

实际MPU驱动+模拟HAL专项（严格警告、ASan/UBSan）通过：首次两个探测超时后恢复、0x69地址回退、持续通信失败/错误WHO_AM_I有界退出、上一轮异常校准拒绝及静止/90°积分。pio run通过：Flash36824 B，RAM3816 B；见build.log。git diff --check通过。未重跑已知存在基线失败的完整导航套件。

本轮新固件未烧录，不能确认实机启动已恢复。若更新后仍为阶段2，需检查IMU3.3V/GND、PB8=SCL/PB9=SDA、上拉和总线通信；软件重试不能证明硬件正常。
