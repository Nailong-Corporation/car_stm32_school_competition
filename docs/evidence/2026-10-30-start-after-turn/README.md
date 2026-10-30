# 2026-10-30 成功转弯后再次无法启动

用户报告又无法启动。按既有要求不核对烧录一致性，只读诊断/IMU/寄存器；未复位、写目标、烧录或操作电机。已detach并退出OpenOCD，原始读取gdb.txt。

## 本轮证据

control_count=6621，button_press_count=3，start_request_count=3，start_result=LINE_START_REJECTED_ROUTE，ROUTE_FAILURE_INVALID。首次stop_snapshot为3892 ms，状态IDLE，以LINE_STOP_ROUTE停车；有效线mask=0x10，目标/PWM为0，未进入行驶。主循环在运行，IMU初始化阶段0、ready=true，不能按此前WHO_AM_I超时处理。本次采样调用栈恰在一次I2C接收内，ErrorCode=0、BUSY_RX为当前传输状态，不能把这一帧BUSY解释为已确认的通信故障。

本地当前路径第三项为{65,112,LEFT,200,0,true}，第二项已由用户调整为200 cm、第三项为65 cm。数字项校验要求0≤tolerance<distance，112≥65使整张表无效，在START时拒绝。没有进行板上表与源码一致性核对，因此归因依据为板上明确的路径非法拒绝、本地非法项及同配置主机复现。

当前后续还锁存TILT，pitch约47.43°，can_restart=false；该状态发生于首次路径拒绝之后，不能覆盖首次原因。IMU偏置-745、当前raw约-6、跨度1584，也提示上电标定阶段应保持静止；本次不改标定门槛或转弯参数。

## 修正与验证

仅将第三项容差112修为12，保留第二段200 cm、第三段65 cm和表速200及其他控制参数；README当前路径示例同步。保持路径校验，不允许非法项进入行驶。

before-test.log：使用保存的route-before.c运行实际生产配置START专项，在test_configured_start.c:27路径有效断言失败。

after-test.log：修正后的实际路径表通过ASan/UBSan专项，在线START接受、无目标线拒绝、编码器正反馈、超速表项与IMU故障门禁保留。

platformio.log：pio run成功，RAM13044/20480 B，Flash38564/65536 B。git diff --check通过。配置笔误修正未改变控制算法，未重复与本次无关的全导航套件；历史全套heading_scale和spatial_short_s失败不宣称已解决。

未烧录、未实车确认启动恢复。后续使用修正固件复测时，应放平车辆并在上电标定期间保持静止；当前板上非法表不能靠重复按PB10修复。
