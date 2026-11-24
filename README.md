# 麦轮底盘

四个M3508加C620电调，走CAN3，ID是1到4。

控制的是速度。先用麦轮运动学算出四个轮子各自的目标转速，每个轮子一个速度环PID算电流发给电调，没有做角度环。

用到队里BSP的PID.h、Motor.h、DJI_Motor.h、Task_Can.h、sysconfig.h，BSP不在这个仓库里。

文件：app_config.h是配置，ID、CAN口、方向、增益、轴距轮距都在这；chassis.c和.h是解算加速度环，把vx、vy、wz换成四个轮子的转速再算电流发出去；chassis_app.c和.h是底盘任务，负责把四个3508注册好、定时跑解算，对外留了设速度和喂反馈的口；pid_app.c和.h管大疆电机的反馈路由。
