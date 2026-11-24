# 麦轮底盘

四个M3508加C620电调，走CAN3，ID是1到4。

控制的是速度。先用麦轮运动学算出四个轮子各自的目标转速，每个轮子一个速度环PID算电流发给电调，没有做角度环。参数都在app_config.h里改。

用到队里BSP的PID.h、Motor.h、DJI_Motor.h、Task_Can.h、sysconfig.h，BSP不在这个仓库里。
