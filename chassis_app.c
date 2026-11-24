/*
 * chassis_app.c：麦轮底盘独立集成层（2026-11-24写）
 *
 * 注册与任务结构照app_glue.c的底盘部分抽出，去掉6020与夹爪，只保留四轮：
 *   - 四台M3508各用独立的pid_type（积分状态不能共用）；
 *   - 方向表、ID、总线、增益、几何全部取app_config.h，本文件不写死值；
 *   - 底盘任务固定周期：Chassis_Solve解算后Chassis_SpeedLoopStep步进并组帧。
 *
 * 状态：未上板验证。增益是借用6020仿真整定的起始值（300、0.5），真机首轮
 * 必须架空、小给定、逐轮整定；几何占位未回填前不要落地跑。
 */

#include "chassis_app.h"
#include "app_config.h"
#include "chassis.h"
#include "pid_app.h"
#include "PID.h"
#include "Motor.h"
#include "DJI_Motor.h"
#include "Task_Can.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* CAN发送队列的唯一定义与创建点（Task_Can.h里是extern声明） */
QueueHandle_t Queue_CANSend = NULL;

motor_t ChassisApp_WheelMotor[4];

static pid_type s_wheel_spd_pid[4];
static motor_t *s_wheel_ptr[4];

/* 底盘速度命令（ChassisApp_SetCmd写、底盘任务读），上电默认静止 */
static volatile float s_cmd_vx = 0.0f;
static volatile float s_cmd_vy = 0.0f;
static volatile float s_cmd_wz = 0.0f;

static void ChassisApp_RegisterWheels(void)
{
    /* 方向表顺序与轮序一致：左前、右前、左后、右后（app_config.h） */
    static const uint8_t dir_tab[4] = {
        APP_WHEEL_DIR_LF, APP_WHEEL_DIR_RF, APP_WHEEL_DIR_LR, APP_WHEEL_DIR_RR
    };
    float para[3] = {APP_CHASSIS_SPD_KP, APP_CHASSIS_SPD_KI, APP_CHASSIS_SPD_KD};
#if (APP_CTRL_PERIOD_MS == 10u)
    para[1] = APP_CHASSIS_SPD_KI * 10.0f;   /* 周期联动换算，与6020同一条规律 */
#endif
    uint8_t i;
    for (i = 0; i < 4; i++)
    {
        PID_Parameter_Init(&s_wheel_spd_pid[i], PID_POSITION, para,
                           M3508OUTMAX, APP_CHASSIS_SPD_MAXIOUT, 0);
        PID_clear(&s_wheel_spd_pid[i]);
        /* PositionPID传NULL：底盘只用速度环（MotortypeRegister只赋值不解引用） */
        MotortypeRegister(&ChassisApp_WheelMotor[i],
                          dir_tab[i] ? FORWARD : REVERSE, MOTOR_3508,
                          NULL, &s_wheel_spd_pid[i],
                          MOTOR3508_Reductionratio,
                          APP_WHEEL_ID_BASE + i, (uint8_t)APP_WHEEL_CANX,
                          0.0f, 0.0f, 0.0f, 360.0f, 0.0f);
        ChassisApp_WheelMotor[i].State = OPEN;
        s_wheel_ptr[i] = &ChassisApp_WheelMotor[i];
    }
}

/* 底盘任务：固定周期解算加四轮速度环步进（周期与pid_app控制任务同口径） */
static void ChassisApp_Task(void *parameters)
{
    float out_rpm[4];
    TickType_t last = xTaskGetTickCount();
    (void)parameters;
    for (;;)
    {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(APP_CTRL_PERIOD_MS));
        Chassis_Solve(s_cmd_vx, s_cmd_vy, s_cmd_wz, out_rpm);
        Chassis_SpeedLoopStep(s_wheel_ptr, (uint8_t)APP_WHEEL_CANX, out_rpm);
    }
}

void ChassisApp_Init(void)
{
    /* 一，队列先行：所有发送都往这个队列投帧，不存在时xQueueSend会Fault */
    if (Queue_CANSend == NULL)
    {
        Queue_CANSend = xQueueCreate(APP_CAN_QUEUE_LEN, sizeof(CanSend_Type));
    }

    /* 二，CAN发送任务：CAN_Init在它内部对三路各调一次，本文件不重复初始化。
     * 队伍完整工程若已有Task_CAN创建点，把APP_CREATE_CAN_TASK改0防重复。 */
#if (APP_CREATE_CAN_TASK == 1u)
    xTaskCreate(Task_CAN, "Task_CAN", APP_TASK_STACK_WORDS, NULL,
                APP_CAN_TASK_PRIO, NULL);
#endif

    /* 三，注册四轮并建底盘任务 */
    ChassisApp_RegisterWheels();
    xTaskCreate(ChassisApp_Task, "Chassis", APP_TASK_STACK_WORDS, NULL,
                APP_CHASSIS_TASK_PRIO, NULL);
}

void ChassisApp_SetCmd(float vx, float vy, float wz)
{
    s_cmd_vx = vx;
    s_cmd_vy = vy;
    s_cmd_wz = wz;
}

uint8_t ChassisApp_FeedDJI(uint32_t stdid, uint8_t adata[])
{
    return DJIFeedback_Route(s_wheel_ptr, 4u, stdid, adata);
}
