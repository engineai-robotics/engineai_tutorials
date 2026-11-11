# Interface Protocol

简体中文 | [English](README.md)

接口协议说明。

## 接口协议

| 话题／服务名 | 类型 | 频率 | 消息 | 说明 |
| --- | --- | --- | --- | --- |
| /hardware/gamepad_keys | 话题发布 | >=500hz | GamepadKeys.msg | 手柄消息反馈 |
| /hardware/imu_info | 话题发布 | >=500hz | IMUInfo.msg | IMU 传感器反馈 |
| /hardware/joint_state | 话题发布 | >=500hz | JointState.msg | 关节状态反馈 |
| /hardware/joint_command | 话题订阅 | 0~500hz | JointCommand.msg | 关节指令订阅 |
| /motion/motion_state | 话题发布 | >=5hz | MotionState.msg | 机器人运动状态反馈 |
| /hardware/motor_debug | 话题发布 | >=50hz | MotorDebug.msg | 温度、力矩等反馈 |
