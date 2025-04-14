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
        STOP,           // 0 停止
        NORMAL,     // 1 正常
        TORQUE,     // 2 扭矩
        RESET,      // 3 复位
        RESET_OVER  // 4 完成复位
    } steerState;

    extern BORAD_TYPE boradType_ ; // 控制板类型(这个任务中只会设置为主驱动轮或辅助驱动轮)
    // TCP_IP_ID : 0 对应 0 
    // TCP_IP_ID : 1 对应 1
    // TCP_IP_ID : 3 对应 2
    // TCP_IP_ID : 4 对应 3
    extern int TCP_IP_ID; // 配置的IP地址的末尾，表示板子的序号，用于后续读取不同的传感器参数值
    void Init();
    void main_assist_board_task(void *pvParameters);
    void main_board_sub_tsk();      // 主驱动轮子任务，主要用于夹紧丝杠的工作
    void assist_board_sub_tsk();    // 辅助驱动轮子任务，主要用于控制两臂家教的丝杠电机
}; 

#endif /* __cplusplus */
#endif /* __MAIN_ASSIST_BORAD_H_ */