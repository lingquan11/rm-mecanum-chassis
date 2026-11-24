#ifndef __CHASSIS_APP_H__
#define __CHASSIS_APP_H__

/*
 * chassis_app：麦轮底盘独立集成层（2026-11-24写）
 *
 * 把chassis.c的解算与速度环接成可跑的整体：创建CAN发送队列与Task_CAN、
 * 注册四台M3508（ID与总线在app_config.h）、建底盘任务，并给出反馈路由
 * 与速度命令两个对外入口。配合的BSP职责边界与app_glue.c同口径：
 * Queue_CANSend在BSP里只有extern声明、由本文件定义并创建（唯一创建点）；
 * CAN_Init由Task_CAN内部完成，本文件不重复调。
 *
 * 状态：未上板验证。几何参数（app_config.h的APP_CHASSIS_LX_M等）是占位值，
 * 轮方向表是中性起点，上地前必须按逐轮点动核对后回填。
 */

#include "sysconfig.h"

/* 初始化：建队列、建CAN任务、注册四轮、建底盘任务。重复调用不重建队列。 */
void ChassisApp_Init(void);

/* 底盘速度命令：vx向右为正、vy向前为正（米每秒），wz逆时针为正（弧度每秒）。
 * 上电默认全0静止；底盘任务每周期读最新值解算。 */
void ChassisApp_SetCmd(float vx, float vy, float wz);

/* 大疆反馈路由入口：在CAN接收路径里对每一帧调用，命中3508反馈帧
 * （0x200加id）则解析并置Online，返回1；不是大疆反馈帧返回0。 */
uint8_t ChassisApp_FeedDJI(uint32_t stdid, uint8_t adata[]);

#endif
