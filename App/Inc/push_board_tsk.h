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

typedef enum
{
    FRONT = 0, // 前侧推杆
    BACK = 1,  // 后侧推杆
    MIDDLE = 2 // 中间推杆
} PUSH_ID;

#define TIM_HZ 50.0 // 定时器的频率，单位Hz

#define USE_VISUAL_PWM 0 // 是否使用模拟PWM输出
static float PUSH_LENGTH[3] = {100.0 , 100.0 , 50.0};       // 推杆的控制长度，单位mm
static float PUSH_LENGTH_MAX[3] = {85.0 , 85.0 , 45.0};   // 推杆的理论工作最大长度，单位mm，用于保护机构
static float PUSH_LENGTH_MIN[3] = {0.0 , 0.0 , 0.0};     // 推杆的理论工作最小长度，单位mm，用于保护机构

namespace TskPushBoard
{
    extern TIM_HandleTypeDef *f_push_tim; // 用于链接前侧推杆的定时器
    extern TIM_HandleTypeDef *b_push_tim; // 用于链接后侧推杆的定时器
    extern TIM_HandleTypeDef *m_push_tim;
    extern uint32_t f_push_channel; // 用于链接前侧推杆的定时器通道
    extern uint32_t b_push_channel; // 用于链接后侧推杆的定时器通道
    extern uint32_t m_push_channel; // 用于链接中间推杆的定时器通道

    void Init();
    void main_push_board_task(void *pvParameters);
    void push_io_init();        // 初始化推杆控制器IO端口
    void set_push_length(float length, uint8_t push_id); // 设置推杆的长度
    // void 
    extern int TCP_IP_ID; // 配置的IP地址的末尾，表示板子的序号，用于后续读取不同的传感器参数值
};

#endif /* __cplusplus */


void TIM3_IRQHandler(void);




#endif
