# 麦轮底盘代码

* 电机：M3508×4＋C620电调，CAN3，ID 1至4
* 控制：麦轮运动学解算出四轮目标转速，每轮一个速度环PID算电流给定，控制周期1ms
* 依赖队伍BSP的`PID.h`、`Motor.h`、`DJI_Motor.h`、`Task_Can.h`，不在本仓库内
* 配置统一在`app_config.h`里改：ID、总线、方向、增益、几何参数

文件：`chassis.c/.h`运动学解算与速度环；`chassis_app.c/.h`底盘任务与四轮注册；`pid_app.c/.h`大疆反馈路由；`app_config.h`配置。
