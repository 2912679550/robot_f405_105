#ifndef __MAIN_ASSIST_BORAD_H_
#define __MAIN_ASSIST_BORAD_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include "main.h"

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include "pid.h"
#include "feedForwardPid.h"
#include "physparams.h"
#include "public_func.h"

namespace TskSteerBoard
{
    typedef enum
    {
        STOP,       // 0 停止
        NORMAL,     // 1 正常
        TORQUE,     // 2 扭矩
        RESET,      // 3 复位
        RESETTING,  // 4 复位中
        RESETOVER,  // 5 复位结束
    } steerState;

    extern BORAD_TYPE boradType_ ; // 控制板类型(这个任务中只会设置为主驱动轮或辅助驱动轮)
    void Init();
    // 根据不同的板子类型装载不同的电机控制参数
    void load_params_();
    // 配置电机的机械参数
    void load_motor_params_();
    // 根据不同的板子类型，更新设置GPIO引脚
    void io_config_();
    void main_assist_board_task(void *pvParameters);
};

#endif /* __cplusplus */
#endif /* __MAIN_ASSIST_BORAD_H_ */