#ifndef __PUSH_BOARD_TSK_H__
#define __PUSH_BOARD_TSK_H__


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

namespace TskPushBoard
{
    void Init();
    void main_push_board_task(void *pvParameters);
};

#endif /* __cplusplus */





#endif
