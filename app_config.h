#ifndef __APP_CONFIG_H__
#define __APP_CONFIG_H__

/*
 * app_config：pack2唯一现场配置点。
 *
 * 原pack的TODO清单里凡是「值」的，全部收在本文件，代码文件里不再留值型TODO。
 * 每项注释含三件事：默认值与口径、来源步号（rehearsal/RUNBOOK.md）、改错的后果一句。
 * 标「占位」或「借用起点」的不是定版值：没在真机实测回填前，不要把注释里的
 * 占位字样删掉冒充定版；每改一项，同步记进rehearsal/PROGRESS.md对应步的回填项。
 */

/* ================= 单台GM6020（阶段1、2） ================= */

/* 6020电机ID，默认1。来源：RUNBOOK步骤1-3。
 * 改错后果：反馈帧0x204加id对不上，FrameCounter不涨、电机永远离线不出力。 */
#define APP_MOTOR6020_ID            1u

/* 6020所在总线，默认CAN1（数值即CANSEND_1至3口径，Task_Can.h）。来源：RUNBOOK步骤1-1。
 * 改错后果：给定发到别的总线，本台不动、同id的别台可能乱动。 */
#define APP_MOTOR6020_CANX          1u

/* 6020控制环形态：1为速度环单环（阶段1），2为角度环串级（阶段2）。来源：RUNBOOK步骤2-1。
 * 改错后果：阶段1还没整定完就切2，两个环一起振荡、无从下手。 */
#define APP_6020_LOOP_MODE          1u

/* 控制任务周期，默认1ms，pid_app与底盘共用，定死后全程不中途改。来源：RUNBOOK步骤1-6、1-7。
 * 联动关系已写进代码：pid_app.h里Ki按周期自动换算（10ms时Ki由0.5自动变5.0，
 * 规律出处SIM-RESULTS.md第2条），不用手改增益；但周期本身改错而电机没跟上，
 * 后果是积分作用差出倍数、稳态误差消不掉（SIM-RESULTS.md第1条实证）。 */
#define APP_CTRL_PERIOD_MS          1u

/* ================= 底盘四台M3508（阶段3） ================= */

/* 底盘总线，默认CAN3，与6020的CAN1分开：四台3508独占一路，反馈0x201至0x204不与
 * 6020的0x205混杂，排故时一眼分清。来源：RUNBOOK步骤3-2。
 * 不选「与6020同总线给2至5」的理由：ID5的电流帧要走0x1FF，与6020电压帧0x1FF
 * 同帧同槽位直接撞车；同总线方案只在ID不超4且槽位排开时才成立，不要照搬。
 * 改错后果：发送帧发错总线，四轮全无反应或与6020抢帧。 */
#define APP_WHEEL_CANX              3u

/* 四轮电机ID起始值，默认1，四轮依次为1、2、3、4（反馈0x201至0x204、电流帧0x200
 * 四个槽位正好排满）。C620拨码必须逐台对上：拨码值等于ID。来源：RUNBOOK步骤3-2。
 * 改错后果：拨码与注册id不一致的那台永远离线，整车会朝一边拽。 */
#define APP_WHEEL_ID_BASE           1u

/* 四轮方向，1为FORWARD、0为REVERSE（Motor.h的MotorDirection_t口径），顺序为
 * 左前、右前、左后、右后。默认全1，只是点动验证前的中性起点。来源：RUNBOOK步骤3-5。
 * 改错后果：某轮反转，车原地打转或斜着跑；点动验完哪轮反就只改哪轮这一项。 */
#define APP_WHEEL_DIR_LF            1u
#define APP_WHEEL_DIR_RF            1u
#define APP_WHEEL_DIR_LR            1u
#define APP_WHEEL_DIR_RR            1u

/* 底盘几何三宏，单位米。默认值0.19、0.19、0.076全是占位假设（152mm麦轮量级），
 * 不是实测。来源：RUNBOOK步骤3-1（量完实车回填）。
 * 改错后果（照抄占位上地）：旋转分量比例失真，原地转一圈角度对不上、复合动作走弧线。 */
#define APP_CHASSIS_LX_M            0.19f   /* 前后半轴距，占位 */
#define APP_CHASSIS_LY_M            0.19f   /* 左右半轮距，占位 */
#define APP_CHASSIS_WHEEL_R_M       0.076f  /* 麦轮半径，占位 */

/* 四轮速度环增益起点：借用6020在仿真里整定的300与0.5（FLOW阶段3既定口径），
 * 只作逐轮整定的第一拍，不是定版。来源：RUNBOOK步骤3-4。
 * 注意：3508反馈是转子侧rpm、量程与6020不同，这组起点在真机上多半偏大，
 * 首轮必须架空、小给定试探。改错后果：四轮一起翻倍调参会互相掩盖问题，
 * 必须逐轮改、逐轮记（步骤3-4纪律）。 */
#define APP_CHASSIS_SPD_KP          300.0f  /* 借用起点 */
#define APP_CHASSIS_SPD_KI          0.5f    /* 借用起点；周期改10ms时同样按乘10换算 */
#define APP_CHASSIS_SPD_KD          0.0f
/* 四轮积分限幅起步值（chassis原骨架口径）。来源：RUNBOOK步骤3-4。
 * 改错后果：给太小会出现6020仿真里MaxIout取5000那种坑，保持转速的稳态输出被钳死。 */
#define APP_CHASSIS_SPD_MAXIOUT     8000u

/* ================= 夹爪DM4310（阶段4） ================= */

/* 夹爪电机id，默认1。总线固定CAN2：达妙专线是BSP硬口径（User_CAN.c的CAN2回调
 * 走FIFO1、DM_driver发送按mot的CAN_ID走CAN2分发），不要改到别的总线。
 * 来源：RUNBOOK步骤4-1。
 * 改错后果：id与驱动板里烧的id不一致，反馈按data[0]低4位匹配不上、FrameCounter不涨。 */
#define APP_GRIPPER_MOTOR_ID        1u
#define APP_GRIPPER_CANX            2u

/* 开合行程标定值，单位度（置零后的连续角度口径）。默认0与60是占位：
 * 60没经置零标定，合位可能没合拢、也可能已经顶死。来源：RUNBOOK步骤4-3。
 * 改错后果：合位角度超过机械行程会持续堵转，靠夹紧监测兜底也不是长久之计。 */
#define APP_GRIP_OPEN_DEG           0.0f    /* 占位，置零标定后回填 */
#define APP_GRIP_CLOSE_DEG          60.0f   /* 占位，置零标定后回填 */

/* 夹紧判定转矩阈值，单位N·m，默认1.5（FLOW阶段4自测起步值，4310额定3至3.5的一半量级）。
 * 来源：RUNBOOK步骤4-6。
 * 改错后果：给太小会把正常加速误判成夹住、合不到位；给太大夹住还硬顶、发烫。 */
#define APP_GRIP_CLAMP_TORQUE_NM    1.5f

/* 速度位置模式的速度上限，单位rad每秒，默认0.5（FLOW阶段4自测起步值）。来源：RUNBOOK步骤4-5。
 * 改错后果：给太大，夹爪到位冲击与堵转电流都变大，机构没验强度前不要放大。 */
#define APP_GRIP_VEL_LIMIT_RAD_S    0.5f

/* 到位判定：角度容差（度）与转速门槛（rad每秒），FLOW阶段4自测起步值。来源：RUNBOOK步骤4-5。 */
#define APP_GRIP_ARRIVE_TOL_DEG     2.0f
#define APP_GRIP_ARRIVE_VEL_RAD_S   0.05f

/* 夹爪任务周期，默认10ms；堵转判定拍数与它配套（30拍约0.3s），改周期必须同比例改拍数。
 * 来源：RUNBOOK步骤4-6。
 * 改错后果：只改周期不改拍数，堵转判定时间跟着变，夹持判定会过早或过晚触发。 */
#define APP_GRIP_PERIOD_MS          10u
#define APP_GRIP_STALL_TICKS        30u

/* ================= 队列与任务（阶段0） ================= */

/* CAN发送队列长度。所给BSP与Task_Can.c里只有extern声明、无创建点（Task_Can.h第25行），
 * 由app_glue.c负责定义并创建，长度在这里定，默认32。来源：RUNBOOK步骤0-8。
 * 改错后果：给太小，高频下发时xQueueSend超时3拍就丢帧，表现为电机偶发不跟随。 */
#define APP_CAN_QUEUE_LEN           32u

/* 是否由app_glue创建Task_CAN任务，默认1。Task_CAN在任务内自调CAN_Init完成三路
 * 滤波与通知激活（Task_Can.c第23至25行），不需要应用层再调CAN_Init。
 * 若队伍完整工程里已有Task_CAN的创建点，把这里改0防重复创建（两个CAN任务抢一个队列）。
 * 来源：RUNBOOK步骤0-8。 */
#define APP_CREATE_CAN_TASK         1u

/* 任务优先级与栈（单位：词）。CAN任务要高于控制任务，保证队列及时排空；
 * 控制任务次之；夹爪最低。栈默认256词，跑起来后用uxTaskGetStackHighWaterMark核。
 * 来源：RUNBOOK步骤0-8（工程口径，BSP未给值）。 */
#define APP_CAN_TASK_PRIO           8u
#define APP_CTRL_TASK_PRIO          7u
#define APP_CHASSIS_TASK_PRIO       7u
#define APP_GRIP_TASK_PRIO          5u
#define APP_TASK_STACK_WORDS        256u

#endif
