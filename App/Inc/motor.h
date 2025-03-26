#ifndef _MOTOR_H_
#define _MOTOR_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include "main.h"

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
#include "main.h"
#include "pid.h"
#include "feedForwardPID.h"
#include "can.h"

typedef enum
{
    PID_MODE_IDLE,
    PID_MODE_POSITION,
    PID_MODE_VELOCITY,
    PID_MODE_TORQUE,
}PID_MODE;

class MOTOR
{
public:
    PID_MODE ctrl_mode; // 0:stop 1:pos 2:vel 3:torque

    float pos_tar;      // 目标位置
    float vel_tar;      // 目标速度
    float cur_tar;      // 目标电流
    
    float pos_current;   // 当前位置
    float vel_current;   // 当前位置的速度
    float cur_current;   // 当前位置的电流
    
    // 从电机转子到pid控制时输入的当前值的转换系数（后续会直接保存到 pos_current中）
    float can2pos; // 位置环输入值转换系数
    float can2vel; // 速度环输入值转换系数
    float can2cur; // 电流环输入值转换系数
    
    // 重置PID
    void resetPosPid();
    void resetVelPid();
    void initMotorPid(PID_PARAM *speedPidParam, FFPID_PARAM *posPidParam);
    // PID计算，输入与期望值的偏差，输出电流值（双环内部会嵌套计算）
    float pidTick(PID_MODE mode, float delta);
    void unpackCanData(moto_measure_t *motorData);
    ffPid* pos_pid = nullptr; // 位置环PID
    Pid* speed_pid = nullptr; // 速度环PID
private:
};

extern MOTOR usedMotors[3]; // 0:dr1 1:dr2 2:dr3

#endif

#endif

