/*
 * pid_app.c：GM6020外环PID应用层（pack2版）
 * 函数签名对齐BSP，不编造BSP不存在的接口。
 * pack2已补完的原TODO：延时实现已定为原生FreeRTOS API（依据见pid_app.h文件头）；
 * 反馈路由落点已由app_glue.c的App_FeedDJI承担；电机ID、总线、周期全部取
 * app_config.h，本文件不再留值型TODO。
 */

#include "pid_app.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"


void PIDApp_6020_Init(motor_t *mot, pid_type *pos_pid, pid_type *spd_pid,
                      uint32_t motor_id, uint8_t canx)
{
    float spd_para[3] = {PIDAPP_SPD_KP, PIDAPP_SPD_KI, PIDAPP_SPD_KD};
    float pos_para[3] = {PIDAPP_POS_KP, PIDAPP_POS_KI, PIDAPP_POS_KD};

    /* MaxOut禁止填0：BSP注释说0为不限制，实现里LIMIT会把输出钳成恒0（见KNOWLEDGE.md第11.2节） */
    PID_Parameter_Init(spd_pid, PID_POSITION, spd_para,
                       PIDAPP_SPD_MAXOUT, PIDAPP_SPD_MAXIOUT, 0);
    PID_Parameter_Init(pos_pid, PID_POSITION, pos_para,
                       PIDAPP_POS_MAXOUT, PIDAPP_POS_MAXIOUT, 0);
    PID_clear(spd_pid);
    PID_clear(pos_pid);

    /* 减速比必须填MOTOR6020_Reductionratio（值为1）：6020无减速箱，
     * 填19或36会使角度与转速口径全盘错位。
     * Kp、Ki、Kd三参数是达妙MIT字段，6020路线不靠它调参，填0。 */
    MotortypeRegister(mot, FORWARD, MOTOR_6020, pos_pid, spd_pid,
                      MOTOR6020_Reductionratio, motor_id, canx,
                      0.0f, 0.0f, 0.0f, 360.0f, 0.0f);
    mot->State = OPEN;
}


uint8_t DJIFeedback_Route(motor_t *motors[], uint8_t count, uint32_t stdid, uint8_t adata[])
{
    uint8_t i;
    for (i = 0; i < count; i++)
    {
        motor_t *mot = motors[i];
        uint32_t expect;
        if (mot == NULL)
        {
            continue;
        }
        if (mot->type == MOTOR_6020)
        {
            expect = M6020_RX_OFFSETID + mot->id;          /* 0x204加id */
        }
        else if (mot->type == MOTOR_3508 || mot->type == MOTOR_2006)
        {
            expect = M3508_M2006_RX_OFFSETID + mot->id;    /* 0x200加id */
        }
        else
        {
            continue;   /* 达妙与瓴控的反馈由BSP的User_CAN.c既有路由分发，不在这里抢 */
        }
        if (stdid == expect)
        {
            DJIMotor_ParaHandle(mot, adata);   /* 内部含首帧初始化与AngleHandle圈数维护 */
            mot->Online = ONLINE;
            return 1;
        }
    }
    return 0;
}
/* 落点已定（pack2）：唯一入口是app_glue.c的App_FeedDJI，它内部调本函数；
 * 往BSP的User_CAN.c三个Received_infoHandle里粘的一行片段见KEIL-READY.md。
 * 原因：HAL回调在BSP的User_CAN.c里，应用层没有现成的帧入口可挂，不改BSP接不进来。 */


void DJIMotor_SendVoltage6020(uint8_t canx, uint32_t stdid, motor_t *group[4])
{
    CanSend_Type tx;
    uint8_t i;
    tx.CANx = canx;
    tx.stdid = stdid;
    for (i = 0; i < 4; i++)
    {
        int16_t out = 0;   /* 空槽必须填0，不要留旧值，否则同组别的电机乱动 */
        if (group[i] != NULL)
        {
            out = group[i]->Out;
        }
        tx.Data[2 * i]     = (uint8_t)((uint16_t)out >> 8);   /* 大端序：高字节在前 */
        tx.Data[2 * i + 1] = (uint8_t)((uint16_t)out & 0xFF);
    }
    /* ID5至7的0x2FF帧只用前6字节，本函数多写的最后2字节不被电机采用，仍按0填出 */
    xQueueSend(Queue_CANSend, &tx, 3);   /* 超时3与BSP各驱动的入队口径一致 */
}


float PIDApp_SingleTurnTarget(const motor_t *mot, float target_deg_single)
{
    float real_total = (float)mot->Realcirnum * 360.0f + mot->RealAngle;
    float base = (float)mot->Realcirnum * 360.0f + target_deg_single;
    while (base - real_total > 180.0f)
    {
        base -= 360.0f;
    }
    while (base - real_total < -180.0f)
    {
        base += 360.0f;
    }
    return base;
}


void Task_6020_SpeedLoop(void *parameters)
{
    PidApp6020Ctx *ctx = (PidApp6020Ctx *)parameters;
    motor_t *mot = ctx->mot;
    TickType_t last = xTaskGetTickCount();
    for (;;)
    {
        float out;
        vTaskDelayUntil(&last, pdMS_TO_TICKS(PIDAPP_CTRL_PERIOD_MS));
        if (mot->Online != ONLINE)
        {
            mot->Out = 0;   /* 反馈离线不出力，防开环冲飞 */
        }
        else
        {
            /* Targetrotationrate对6020就是rpm口径：DJIMotor_ParaHandle把RealSpeed
             * 原样写进该字段（字段注释写rps，以解析代码为准，见KNOWLEDGE.md第11.3节） */
            out = Pid_cal(mot->SpeedPID, mot->Targetrotationrate,
                          (float)mot->FeedbackData.RealSpeed);
            mot->Out = (int16_t)out;
        }
        DJIMotor_SendVoltage6020(ctx->canx, ctx->tx_stdid, ctx->group);
    }
}


void Task_6020_Cascade(void *parameters)
{
    PidApp6020Ctx *ctx = (PidApp6020Ctx *)parameters;
    motor_t *mot = ctx->mot;
    TickType_t last = xTaskGetTickCount();
    for (;;)
    {
        float real_total, target_total, target_rpm, out;
        vTaskDelayUntil(&last, pdMS_TO_TICKS(PIDAPP_CTRL_PERIOD_MS));
        if (mot->Online != ONLINE)
        {
            mot->Out = 0;
            DJIMotor_SendVoltage6020(ctx->canx, ctx->tx_stdid, ctx->group);
            continue;
        }
        /* 多圈口径：Realcirnum由DJIMotor_AngleHandle维护，不要另写判圈逻辑 */
        real_total = (float)mot->Realcirnum * 360.0f + mot->RealAngle;
        target_total = (float)mot->Targetcirnum * 360.0f + mot->TargetAngle;
        /* 单圈定角时，应用层先用PIDApp_SingleTurnTarget换算target_total再写回
         * Targetcirnum与TargetAngle，或直接在此处替换target_total，二选一、不要叠加 */
        target_rpm = Pid_cal(mot->PositionPID, target_total, real_total);
        out = Pid_cal(mot->SpeedPID, target_rpm,
                      (float)mot->FeedbackData.RealSpeed);
        mot->Out = (int16_t)out;
        DJIMotor_SendVoltage6020(ctx->canx, ctx->tx_stdid, ctx->group);
    }
}
