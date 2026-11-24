#ifndef __CHASSIS_H__
#define __CHASSIS_H__

/*
 * chassis：麦轮底盘解算与四轮速度环步进（pack2版，配合FLOW.md阶段3）
 * 本头文件是pack2为chassis.c补的声明文件，函数签名逐字取自chassis.c原骨架，
 * 不是BSP的头文件，不要与BSP混淆。
 * 四台3508的注册与底盘任务本体在app_glue.c；几何与增益值在app_config.h。
 */

#include "sysconfig.h"
#include "Motor.h"

/* 正运动学解算：输入vx、vy为米每秒、wz为弧度每秒，输出四轮输出轴转速rpm
 * （轮序：0左前、1右前、2左后、3右后，约定见chassis.c文件头） */
void Chassis_Solve(float vx, float vy, float wz, float wheel_out_rpm[4]);

/* 输出轴rpm换转子侧rpm（乘MOTOR3508_Reductionratio） */
float Chassis_MotorRpm(float out_rpm);

/* C620电流模式组帧发送：帧号0x200，四轮Out各占2字节大端序，投入Queue_CANSend */
void Chassis_SendCurrent(uint8_t canx, motor_t *wheel[4]);

/* 四轮速度环步进一次：target_out_rpm为Chassis_Solve输出（输出轴rpm），
 * 内部换转子口径喂Pid_cal，最后调Chassis_SendCurrent发一帧 */
void Chassis_SpeedLoopStep(motor_t *wheel[4], uint8_t canx, const float target_out_rpm[4]);

#endif
