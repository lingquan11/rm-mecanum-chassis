/*
 * chassis.c：麦轮底盘解算与四轮速度环步进（pack2版，配合FLOW.md阶段3）
 *
 * 已确认：底盘电机为M3508（用户2026-11-13确认），电调C620为RM标准搭配。
 * 接口对齐BSP真实签名：Pid_cal / CanSend_Type / Queue_CANSend；反馈解析走
 * pid_app.c的DJIFeedback_Route（3508反馈帧0x200加id已在该函数覆盖，
 * 底盘不要再写一份路由）。
 *
 * pack2已补完的原TODO：四台3508的注册与各自PID初始化、底盘任务本体已由
 * app_glue.c承担；几何、轮位ID、方向、增益全部收进app_config.h，
 * 本文件不再留值型TODO。
 *
 * 轮序与几何约定（自测约定，真机必须按FLOW阶段3逐轮点动核对）：
 *   轮编号：0左前、1右前、2左后、3右后；辊子呈X型布置；
 *   车体坐标：vx向右为正、vy向前为正、wz逆时针为正（右手系，z向上）；
 *   几何参数取app_config.h，占位值未回填前不要上地跑。
 */

#include "chassis.h"
#include "sysconfig.h"
#include "PID.h"
#include "Motor.h"
#include "DJI_Motor.h"
#include "Task_Can.h"
#include "app_config.h"
#include "queue.h"

/* ---- 几何参数：唯一来源是app_config.h（占位假设，实车实测后在那边改） ---- */
#define CHASSIS_LX_M        APP_CHASSIS_LX_M
#define CHASSIS_LY_M        APP_CHASSIS_LY_M
#define CHASSIS_WHEEL_R_M   APP_CHASSIS_WHEEL_R_M

/* 四轮速度环限幅与增益不在本文件：MaxOut用DJI_Motor.h的M3508OUTMAX（15999，
 * BSP代码口径），MaxIout与Kp、Ki在app_config.h，初始化在app_glue.c。
 * 四轮必须各用独立的pid_type，积分状态共用会互相污染。 */

enum
{
    WHEEL_LF = 0,
    WHEEL_RF = 1,
    WHEEL_LR = 2,
    WHEEL_RR = 3
};

/*
 * 正运动学解算（本骨架选定的X型约定）：
 *   vLF = vy - vx - wz*(Lx+Ly)     vRF = vy + vx + wz*(Lx+Ly)
 *   vLR = vy + vx - wz*(Lx+Ly)     vRR = vy - vx + wz*(Lx+Ly)
 * 自检性质：纯vy时四轮等值同向；纯wz时左右两侧反向；纯vx时对角组合反向。
 * vx项的对角组合方向取决于辊子实际朝向：真机架空点动时若纯平移方向镜像，
 * 只翻vx项符号（或等价地调轮方向表），不要改公式其余结构。
 * 输入：vx、vy为米每秒，wz为弧度每秒。输出：四轮输出轴转速，rpm。
 */
void Chassis_Solve(float vx, float vy, float wz, float wheel_out_rpm[4])
{
    float k = wz * (CHASSIS_LX_M + CHASSIS_LY_M);   /* 旋转在轮缘的线速度，米每秒 */
    float v[4];
    v[WHEEL_LF] = vy - vx - k;
    v[WHEEL_RF] = vy + vx + k;
    v[WHEEL_LR] = vy + vx - k;
    v[WHEEL_RR] = vy - vx + k;
    for (int i = 0; i < 4; i++)
    {
        /* 线速度换输出轴rpm：除以周长得转每秒，再乘60 */
        wheel_out_rpm[i] = v[i] / (2.0f * 3.14159265f * CHASSIS_WHEEL_R_M) * 60.0f;
    }
}

/*
 * 输出轴rpm换转子侧rpm：速度环反馈FeedbackData.RealSpeed是转子侧rpm
 *（DJIMotor_ParaHandle原样写入、未除减速比），给定必须同口径。
 * 减速比用BSP宏MOTOR3508_Reductionratio（代码取整19，手册19.2、约1%误差）。
 */
float Chassis_MotorRpm(float out_rpm)
{
    return out_rpm * (float)MOTOR3508_Reductionratio;
}

/* C620电流模式组帧：帧号0x200（M3508_M2006_CH1_TX_OFFSETID），ID1至4各占2字节大端序 */
void Chassis_SendCurrent(uint8_t canx, motor_t *wheel[4])
{
    CanSend_Type tx;
    tx.CANx = canx;
    tx.stdid = M3508_M2006_CH1_TX_OFFSETID;
    for (int i = 0; i < 4; i++)
    {
        int16_t out = 0;   /* 空槽填0 */
        if (wheel[i] != NULL)
        {
            out = wheel[i]->Out;
        }
        tx.Data[2 * i]     = (uint8_t)((uint16_t)out >> 8);
        tx.Data[2 * i + 1] = (uint8_t)((uint16_t)out & 0xFF);
    }
    xQueueSend(Queue_CANSend, &tx, 3);
}

/*
 * 四轮速度环步进一次（由app_glue.c的底盘任务按固定周期调用，周期口径与
 * FLOW阶段1一致）：target_out_rpm为Chassis_Solve的输出（输出轴rpm），
 * 内部换转子口径再喂Pid_cal。注册与PID初始化在app_glue.c的App_RegisterWheels：
 * type填MOTOR_3508、减速比填MOTOR3508_Reductionratio、MaxOut填M3508OUTMAX；
 * 轮位与电机id对应关系在app_config.h，点动验证通过前不要落地。
 */
void Chassis_SpeedLoopStep(motor_t *wheel[4], uint8_t canx, const float target_out_rpm[4])
{
    for (int i = 0; i < 4; i++)
    {
        if (wheel[i] == NULL || wheel[i]->Online != ONLINE)
        {
            if (wheel[i] != NULL)
            {
                wheel[i]->Out = 0;   /* 离线轮不出力 */
            }
            continue;
        }
        float target_rotor = Chassis_MotorRpm(target_out_rpm[i]);
        float out = Pid_cal(wheel[i]->SpeedPID, target_rotor,
                            (float)wheel[i]->FeedbackData.RealSpeed);
        wheel[i]->Out = (int16_t)out;
    }
    Chassis_SendCurrent(canx, wheel);
}
