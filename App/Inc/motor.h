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
#define MOTOR_NUM 3 // 电机数量


typedef enum
{
    PID_MODE_IDLE,
    PID_MODE_POSITION,
    PID_MODE_VELOCITY,
    PID_MODE_TORQUE,
}PID_MODE;

typedef enum
{
    NORMAL, // 正常工作
    BLOCK,  // 电机堵转
} WORKING_LOG;

class MOTOR
{
public:
    PID_MODE ctrl_mode = PID_MODE::PID_MODE_IDLE ; // 0:stop 1:pos 2:vel 3:torque
    WORKING_LOG work_log = WORKING_LOG::NORMAL; // 电机工作状态

    bool outer_pos = false; // 配置电机是否使用外部传感器的测量值作为位置值
    bool outer_vel = false; // 配置电机是否使用外部传感器的测量值作为速度值

    float mech_dir = 1.0f; // 电机安装后正反转在机械数学定义上的方向

    int block_cnt = 0; // 堵转计数器

    float pos_tar;      // 目标位置
    float pos_current;   // 当前位置

    float vel_tar;      // 目标速度
    float vel_current;   // 当前位置的速度
    
    float cur_tar;      // 目标电流
    float cur_current;   // 当前位置的电流
    
    // 从电机转子到pid控制时输入的当前值的转换系数（后续会直接保存到 pos_current中）
    float can2pos; // 位置环输入值转换系数
    float can2vel; // 速度环输入值转换系数
    float can2cur; // 电流环输入值转换系数

    // 从标定后的零点值到数学定义上的零点值的偏置
    float offset_angle; // 电机的偏置角度值
    bool cali_flag = false; // 是否已经标定过
    
    // 重置PID
    void resetPosPid();
    void resetVelPid();
    void initMotorPid(PID_PARAM *speedPidParam, FFPID_PARAM *posPidParam);
    // PID计算，输入与期望值的偏差，输出电流值（双环内部会嵌套计算）
    float pidTick(PID_MODE mode, float delta);
    void unpackCanData(moto_measure_t *motorData);
    ffPid* pos_pid = nullptr; // 位置环PID
    Pid* speed_pid = nullptr; // 速度环PID
    void set_tar(PID_MODE mode, float tar) ;
    void set_delta(PID_MODE mode, float delta) ;
    void set_cali_val(float offset);

private:
    int block_cur = 5.0f; // 堵转电流值
    int block_max = 20; // 堵转计数上限：连续200次期望电流值超过设定的堵转电流值，则认为电机堵转
};

extern MOTOR usedMotors[MOTOR_NUM]; // 0:dr1 1:dr2 2:dr3

#endif

#endif

