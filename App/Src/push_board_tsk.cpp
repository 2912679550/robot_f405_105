#include "push_board_tsk.h"
#include "ethernet_tsk.h"
#include "main.h"
#include "public_func.h"


#define push_debug 0
bool pcOnline_ = false; // 连接上PC的标志位
float debugTarLength[3] = {30.0f, 30.0f, 20.0f}; // 用于调试的目标长度,便于在debug中直接控制

namespace TskPushBoard
{
    const int tskStkSize = 512;
    uint8_t tskPeriod = 5; // ms
    TIM_HandleTypeDef* f_push_tim = &htim4; // 用于链接前侧推杆的定时器
    TIM_HandleTypeDef* b_push_tim = &htim4; // 用于链接后侧推杆的定时器
    TIM_HandleTypeDef* m_push_tim = &htim3;
    uint32_t f_push_channel = TIM_CHANNEL_1; // 用于链接前侧推杆的定时器通道
    uint32_t b_push_channel = TIM_CHANNEL_2; // 用于链接后侧推杆的定时器通道
    uint32_t m_push_channel = TIM_CHANNEL_3; // 用于链接中间推杆的定时器通道
    TIM_HandleTypeDef* allTims[3] = {f_push_tim, b_push_tim, m_push_tim};
    uint32_t allChannels[3] = {f_push_channel, b_push_channel, m_push_channel};

    // * 创建指令信道容器
    PUSH_CMD *pushCmd_ = nullptr; // 用于控制推杆的命令
    PUSH_VAL *pushVal_ = nullptr; // 用于反馈推杆的状态
    int TCP_IP_ID = 0; // 配置的IP地址的末尾，表示板子的序号，用于后续读取不同的传感器参数值

    void Init(){
        BaseType_t rtn;
        // 分配内存
        pushCmd_ = (PUSH_CMD *)pvPortMalloc(sizeof(PUSH_CMD));
        if (pushCmd_ == nullptr)
            return;
        // 给一个命令的初始值
        pushCmd_->tar_length_f = 30.f; // 前侧推杆的目标长度
        pushCmd_->tar_length_b = 30.f; // 后侧推杆的目标长度
        pushCmd_->tar_length_m = 20.f; // 中间推杆的目标长度

        pushVal_ = (PUSH_VAL *)pvPortMalloc(sizeof(PUSH_VAL));
        if (pushVal_ == nullptr)
            return;
        // 初始化PWM输出IO
        push_io_init();
        // 创建线程任务
        rtn = xTaskCreate(main_push_board_task, (const portCHAR *)"pushTask",
                          tskStkSize, NULL, osPriorityAboveNormal, NULL);
        configASSERT(rtn == pdPASS);
    }

    void main_push_board_task(void *pvParameters)
    {
        while (true)
        {
            vTaskDelay(tskPeriod); // 5ms
            BaseType_t rtn;
            // * 接收控制指令
            rtn = xQueueReceive(TskEth::pushCmdQueue, pushCmd_, 0);
            if (rtn == pdPASS)
            {
                pcOnline_ = true; // 连接上PC的标志位
            }
            #if push_debug
            // * 在Debug模式中直接使用数组来控制推杆长度，便于调试
            set_push_length(debugTarLength[0], PUSH_ID::FRONT); // 设置前侧推杆的长度
            set_push_length(debugTarLength[1], PUSH_ID::BACK);  // 设置后侧推杆的长度
            set_push_length(debugTarLength[2], PUSH_ID::MIDDLE); // 设置中间推杆的长度
            #else
            if (pcOnline_ == false){
                set_push_length(debugTarLength[0], PUSH_ID::FRONT); // 设置前侧推杆的长度
                set_push_length(debugTarLength[1], PUSH_ID::BACK);  // 设置后侧推杆的长度
                set_push_length(debugTarLength[2], PUSH_ID::MIDDLE); // 设置中间推杆的长度
                continue;
            }
            else{
                // * 设置推杆的长度
                set_push_length(pushCmd_->tar_length_f, PUSH_ID::FRONT); // 设置前侧推杆的长度
                set_push_length(pushCmd_->tar_length_b, PUSH_ID::BACK);  // 设置后侧推杆的长度
                set_push_length(pushCmd_->tar_length_m, PUSH_ID::MIDDLE); // 设置中间推杆的长度
            }
            // * 回传推杆当前长度
            pushVal_->cur_length_b = 0.0f;
            pushVal_->cur_length_f = 0.0f;
            pushVal_->cur_length_m = 0.0f;
            xQueueSend(TskEth::pushValQueue, pushVal_, 0); // 发送当前推杆的长度
            #endif
        }
        
    }

    void set_push_length(float length, uint8_t push_id){
        // 设置推杆的长度
        // 1. 计算对应的PWM值
        // 2. 设置PWM值
        if(push_id > PUSH_ID::MIDDLE) return; // 超出范围，直接返回
        if(length < PUSH_LENGTH_MIN[push_id]) length = PUSH_LENGTH_MIN[push_id]; // 小于0，设置为0
        if(length > PUSH_LENGTH_MAX[push_id]) length = PUSH_LENGTH_MAX[push_id]; // 大于最大值，设置为最大值
        float pwm_value = (length / PUSH_LENGTH[push_id]) * 1000.0f + 1000.0f; // 计算对应的PWM值
        __HAL_TIM_SET_COMPARE(allTims[push_id], allChannels[push_id], (uint32_t)pwm_value); // 设置PWM值
    }

    void push_io_init(){
        // 用于初始化推杆PWM输出控制器的端口
        // 由于IO数量不足，目前打算复用串口的两个引脚作为推杆的PWM输出控制器
        // 前： PB6 对应串口的TXD引脚 TIM4_CH1
        // 后： PB7 对应串口的RXD引脚 TIM4_CH2
        // 中： PB0 TIM3_CH3
        // * 引脚设置
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        __HAL_RCC_GPIOB_CLK_ENABLE(); // 使能GPIOB时钟
        GPIO_InitStruct.Pin = GPIO_PIN_0; // PB0引脚
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; // 设置为复用推挽输出模式
        GPIO_InitStruct.Pull = GPIO_PULLUP; // 无上下拉电阻
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; // 高速模式
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM3; // 复用为TIM3_CH3
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); // 初始化PB0引脚
        GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7; // PB6和PB7引脚
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM4; // 复用为TIM4_CH1和TIM4_CH2
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); // 初始化PB6和PB7引脚
        
        // todo 首先配置定时器3
        // * 定时参数设置
        __HAL_RCC_TIM3_CLK_ENABLE(); // 使能TIM3时钟
        TIM_OC_InitTypeDef sConfigOC = {0};
        // 初始化定时器3,频率为50Hz，计数范围为20ms
        htim3.Instance = TIM3;
        htim3.Init.Prescaler = 84-1;                    // 预分频器设置为84-1，定时器时钟频率为84MHz
        htim3.Init.CounterMode = TIM_COUNTERMODE_UP;    // 计数器向上计数
        htim3.Init.Period =  int(1 / TIM_HZ * 1000.0 * 1000.0 - 1); // 自动重装载寄存器值
        htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // 时钟分频器设置为1
        htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; // 自动重装载预加载使能
        HAL_TIM_PWM_Init(m_push_tim); // 初始化定时器3
        // 配置定时器3的输出通道TIM3_CH3，链接到PB0引脚
        sConfigOC.OCMode = TIM_OCMODE_PWM1; // 设置输出比较模式为PWM1
        sConfigOC.Pulse = 1000; // 对应高电平时间为1ms
        sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH; // 输出极性为高电平有效
        sConfigOC.OCFastMode = TIM_OCFAST_DISABLE; // 快速模式禁止
        HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, m_push_channel); // 配置定时器3的输出通道TIM3_CH3
        HAL_TIM_PWM_Start(m_push_tim, m_push_channel); // 启动定时器3的PWM输出
        
        // todo 然后配置定时器4
        __HAL_RCC_TIM4_CLK_ENABLE(); // 使能TIM4时钟
        htim4.Instance = TIM4;
        htim4.Init.Prescaler = 84 - 1;
        htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
        htim4.Init.Period =int( 1 / TIM_HZ * 1000.0 * 1000.0 - 1 ); // 自动重装载寄存器值
        htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; // 自动重装载预加载使能
        HAL_TIM_PWM_Init(f_push_tim); // 初始化定时器4
        // 配置定时器4的输出通道TIM4_CH1和TIM4_CH2，链接到PB6和PB7引脚
        sConfigOC.OCMode = TIM_OCMODE_PWM1; // 设置输出比较模式为PWM1
        sConfigOC.Pulse = 1000; // 对应高电平时间为1ms
        sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH; // 输出极性为高电平有效
        sConfigOC.OCFastMode = TIM_OCFAST_DISABLE; // 快速模式禁止
        HAL_TIM_PWM_ConfigChannel(f_push_tim, &sConfigOC, f_push_channel); // 配置定时器4的输出通道TIM4_CH1
        HAL_TIM_PWM_ConfigChannel(f_push_tim, &sConfigOC, b_push_channel); // 配置定时器4的输出通道TIM4_CH2
        HAL_TIM_PWM_Start(f_push_tim, f_push_channel); // 启动定时器4的PWM输出TIM4_CH1
        HAL_TIM_PWM_Start(f_push_tim, b_push_channel); // 启动定时器4的PWM输出TIM4_CH2
    }
}