#include "main_assist_board_tsk.h"
#include "ethernet_tsk.h"
#include "math.h"

namespace TskSteerBoard
{
    const int tskStkSize = 512;
    SteerCmd *steerCmd_ = nullptr;       // 用于控制舵轮的命令
    SteerRunValue *steerVal_ = nullptr;  // 用于反馈舵轮的状态
    moto_measure_t *motor_ = nullptr;
    BORAD_TYPE boradType_ = BORAD_TYPE::idle; // 控制板类型(这个任务中只会设置为主驱动轮或辅助驱动轮)

    uint32_t steerCnt = 0;
    // 判断是主驱动轮控制板还是辅助驱动轮控制板
    // 但主驱动轮和辅助驱动轮的一些参数数值不一样，需要在程序中区分装载

    void Init()
    {
        BaseType_t rtn;
        // 为指针指向的Class分配内存
        steerCmd_ = (SteerCmd *)pvPortMalloc(sizeof(SteerCmd));
        if (steerCmd_ == nullptr)
            return;
        steerVal_ = (SteerRunValue *)pvPortMalloc(sizeof(SteerRunValue));
        if (steerVal_ == nullptr)
            return;
        motor_ = (moto_measure_t *)pvPortMalloc(sizeof(moto_measure_t));
        if (motor_ == nullptr)
            return;
        // Create tasks
        rtn = xTaskCreate(main_assist_board_task, (const portCHAR *)"steerTask",
                          tskStkSize, NULL, osPriorityAboveNormal, NULL);
        configASSERT(rtn == pdPASS);
    }

    void load_params_(){

    }

    void main_assist_board_task(void *pvParameters)
    {
        BaseType_t rtn;
        can1RxQueueHandle = xQueueCreate(4, sizeof(moto_measure_t));

        // 初始化CAN并使能中断
        CAN_Start_Trans();
    }



};