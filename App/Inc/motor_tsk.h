// Author: Vulcan
// Date: 2025.0407
// Description: 用于电机自动控制与PID计算的线程任务头文件声明

#ifndef MOTOR_TSK_H_
#define MOTOR_TSK_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include "main.h"
#include "motor.h"
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include "pid.h"
#include "feedForwardPid.h"
#include "physparams.h"
#include "public_func.h"
#include "main.h"

namespace TskMotorPID{
    extern BORAD_TYPE boradType_ ; // 控制板类型(这个任务中只会设置为主驱动轮或辅助驱动轮)
    extern int TCP_IP_ID; // 配置的IP地址的末尾，表示板子的序号，用于后续读取不同的传感器参数值
    void Init();
    void motorTickTask(void *pvParameters);
    void load_pid_params_(); // 读取参数
    void load_motor_params_(); // 装载电机类的机械参数
}

#endif // __cplusplus

#endif  /* MOTOR_TSK_H_ */