
#ifndef __STEERWHEEL_TSK_H_
#define __STEERWHEEL_TSK_H_

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

namespace TskSteer
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

    void Init();
};

#endif /* __cplusplus */
#endif /* __STEERWHEEL_TSK_H_ */