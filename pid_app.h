#ifndef __PID_APP_H__
#define __PID_APP_H__

/*
 * pid_app：GM6020外环PID应用层（pack2版，配合FLOW.md阶段1与阶段2）
 *
 * 接口全部对齐队伍BSP的真实签名：
 *   PID.h      ：PID_Parameter_Init / PID_Parameter_Update / PID_clear / Pid_cal
 *   Motor.h    ：motor_t / MotortypeRegister
 *   DJI_Motor.h：DJIMotor_ParaHandle（内部会调DJIMotor_AngleHandle维护圈数）
 *   Task_Can.h ：CanSend_Type / Queue_CANSend（extern声明）/ CANSEND_1至3
 * BSP里不存在的函数本文件不编造。
 *
 * pack2相对pack的变化（值型TODO全部收进app_config.h）：
 *   - 控制周期不在本文件写死，取app_config.h的APP_CTRL_PERIOD_MS；
 *   - 速度环Ki按周期联动换算已写进本文件（见下方PIDAPP_SPD_KI），改周期只改
 *     app_config一处、不用手改增益；
 *   - 反馈路由落点已由app_glue.c的App_FeedDJI承担，粘贴片段见KEIL-READY.md；
 *   - 延时实现已定：原生FreeRTOS API可用（sysconfig.h第17至18行已含cmsis_os.h
 *     与task.h），vTaskDelayUntil不用换CMSIS封装。
 *
 * 增益为仿真起始值（见rehearsal/SIM-RESULTS.md），真机须再调。
 * 调用周期定死：Pid_cal无周期归一化，改周期必须按比例换算Ki与Kd（Ki已自动处理）。
 */

#include "sysconfig.h"
#include "PID.h"
#include "Motor.h"
#include "DJI_Motor.h"
#include "Task_Can.h"
#include "app_config.h"

/* ---- 控制任务周期：唯一来源是app_config.h ---- */
#define PIDAPP_CTRL_PERIOD_MS    APP_CTRL_PERIOD_MS

/* ---- 速度环起始增益（电压模式，仿真起点） ---- */
#define PIDAPP_SPD_KP            300.0f
#if (APP_CTRL_PERIOD_MS == 10u)
#define PIDAPP_SPD_KI            5.0f    /* 周期10ms时Ki乘10，规律出处SIM-RESULTS.md第2条 */
#else
#define PIDAPP_SPD_KI            0.5f    /* 周期1ms基线（SIM-RESULTS.md速度环基线组） */
#endif
#define PIDAPP_SPD_KD            0.0f
#define PIDAPP_SPD_MAXOUT        15000u  /* 调试限幅；确认安全后放大，不超M6020OutMax的29999 */
#define PIDAPP_SPD_MAXIOUT       10000u  /* 按保持转速所需稳态输出估，勿照三分之一拍小 */

/* ---- 角度环（串级外环）起始增益 ---- */
#define PIDAPP_POS_KP            2.0f
#define PIDAPP_POS_KI            0.0f
#define PIDAPP_POS_KD            0.0f
#define PIDAPP_POS_MAXOUT        120u    /* 单位rpm：外环输出即目标转速上限 */
#define PIDAPP_POS_MAXIOUT       30u

/* 一个6020控制单元的上下文：单台测试时group里只放本台与其同组的电机 */
typedef struct
{
    motor_t *mot;          /* 本台电机 */
    motor_t *group[4];     /* 同一发送帧内的电机（槽位序与ID序一致），空槽填NULL */
    uint32_t tx_stdid;     /* 电压模式：ID1至4填0x1FF（M6020_CH1_TX_OFFSETID），ID5至7填0x2FF */
    uint8_t  canx;         /* CANSEND_1至3，电机挂哪路FDCAN填哪路 */
} PidApp6020Ctx;

/* 注册并初始化一台6020（内部调MotortypeRegister与两环的PID_Parameter_Init、PID_clear） */
void PIDApp_6020_Init(motor_t *mot, pid_type *pos_pid, pid_type *spd_pid,
                      uint32_t motor_id, uint8_t canx);

/*
 * 反馈路由补调用（补BSP缺口：DJIMotor_ParaHandle在BSP内无调用点）。
 * 在应用层接收路径里对每一帧调用：按stdid匹配6020的0x204加id、
 * 3508与2006的0x200加id，命中则调DJIMotor_ParaHandle解析并置Online。
 * 返回1表示命中并已解析，0表示不是大疆反馈帧。
 * pack2唯一落点是app_glue.c的App_FeedDJI，往User_CAN.c里粘的片段见KEIL-READY.md。
 */
uint8_t DJIFeedback_Route(motor_t *motors[], uint8_t count, uint32_t stdid, uint8_t adata[]);

/* 把group四台电机的Out按槽位大端序组一帧电压给定，投入Queue_CANSend */
void DJIMotor_SendVoltage6020(uint8_t canx, uint32_t stdid, motor_t *group[4]);

/* 单圈定角辅助：把0至360内的单圈目标换算成与当前多圈口径等效的target_total（度，走最短路径） */
float PIDApp_SingleTurnTarget(const motor_t *mot, float target_deg_single);

/* 控制任务：速度环单环（阶段1）。参数传PidApp6020Ctx指针，任务内vTaskDelayUntil固定周期 */
void Task_6020_SpeedLoop(void *parameters);

/* 控制任务：角度环串级（阶段2）。参数传PidApp6020Ctx指针，周期与速度环相同 */
void Task_6020_Cascade(void *parameters);

#endif
