# 麦轮底盘代码

## 电机与电调

- 电机：M3508×4，电调C620
- 总线：CAN3，电机ID 1、2、3、4（反馈帧0x201至0x204，电流给定帧0x200）
- 轮序：左前、右前、左后、右后，辊子X型布置
- 减速比：MOTOR3508_Reductionratio（19）

## 控制方式

- 只有速度环，没有角度环：解算出四轮目标转速后，每轮一个位置式PID算电流给定
- 增益起始值：Kp=300，Ki=0.5，Kd=0（借用6020仿真整定值，未在3508上整定）
- 输出限幅：MaxOut=15999（M3508OUTMAX），MaxIout=8000
- 控制周期：1ms（改周期要按比例换算Ki）
- 运动学：输入vx（向右）、vy（向前）、wz（逆时针），单位m/s、m/s、rad/s，输出四轮输出轴rpm，内部换转子侧rpm再喂PID

## 依赖的BSP文件（队伍BSP，非本仓库内）

- `PID.h`：Pid_cal、PID_Parameter_Init、PID_clear
- `Motor.h`：motor_t、MotortypeRegister
- `DJI_Motor.h`：DJIMotor_ParaHandle（反馈解析，pid_app.c调用）
- `Task_Can.h`：CanSend_Type、Queue_CANSend、Task_CAN
- `sysconfig.h`、FreeRTOS的`task.h`、`queue.h`

## 本仓库文件

- `chassis.c/.h`：运动学解算、四轮速度环步进、C620组帧发送
- `chassis_app.c/.h`：四轮注册、底盘任务、速度命令与反馈路由入口
- `pid_app.c/.h`：大疆反馈路由DJIFeedback_Route
- `app_config.h`：唯一配置点（ID、总线、方向、增益、几何）
