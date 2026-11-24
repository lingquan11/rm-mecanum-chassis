# 麦轮底盘代码

## 电机与电调

* **硬件**：M3508×4+C620电调，减速比按`MOTOR3508_Reductionratio`（19）走。
* **总线**：走CAN3，四个电机ID依次为1、2、3、4（反馈帧`0x201`到`0x204`，电流给定帧`0x200`）。
* **轮序**：按左前、右前、左后、右后排，辊子X型布置。

## 控制方式

* **单速度环**：只做速度环，不加角度环。运动学解算出四轮目标转速后，每轮各跑一个位置式PID算电流给定。
* **增益起点**：`Kp=300`、`Ki=0.5`、`Kd=0`——这组数直接借了6020的仿真整定值，还没在3508上整定。
* **输出限幅**：`MaxOut=15999`（`M3508OUTMAX`）、`MaxIout=8000`。
* **控制周期**：默认1ms；后面改周期的话，记得按比例同步换算`Ki`。
* **运动学链路**：输入`vx`（向右）、`vy`（向前）、`wz`（逆时针），单位依次为`m/s`、`m/s`、`rad/s`→先算四轮输出轴rpm→内部换算到转子侧rpm，再喂进PID。

## 依赖的BSP文件（队伍BSP，不在本仓库内）

* `PID.h`：`Pid_cal`、`PID_Parameter_Init`、`PID_clear`
* `Motor.h`：`motor_t`、`MotortypeRegister`
* `DJI_Motor.h`：`DJIMotor_ParaHandle`（反馈解析，由`pid_app.c`调用）
* `Task_Can.h`：`CanSend_Type`、`Queue_CANSend`、`Task_CAN`
* 系统与RTOS：`sysconfig.h`，以及FreeRTOS的`task.h`、`queue.h`

## 本仓库文件

* `chassis.c/.h`：运动学解算、四轮速度环步进+C620组帧发送
* `chassis_app.c/.h`：四轮注册、底盘任务，以及速度命令和反馈路由入口
* `pid_app.c/.h`：大疆反馈路由`DJIFeedback_Route`
* `app_config.h`：唯一配置入口，ID、总线、方向、增益和几何参数都在这里改
