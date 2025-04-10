#include "main_assist_board_tsk.h"
#include "ethernet_tsk.h"
#include "math.h"
#include "motor.h"
#include "adcSensor.h"
#include "public_func.h"

uint16_t adcOri[3] = {0};
float adcVal[3] = {0};

namespace TskSteerBoard
{
    const int tskStkSize = 512;
    // * 控制指令
    uint8_t tskPeriod = 5; // ms
    MAIN_ASSIST_CMD *boardCmd_ = nullptr; // 用于控制辅助驱动轮的命令
    MAIN_ASSIST_VAL *boardVal_ = nullptr; // 用于反馈辅助驱动轮的状态

    // * 信息容器
    BORAD_TYPE boradType_ = BORAD_TYPE::idle; // 控制板类型(这个任务中只会设置为主驱动轮或辅助驱动轮)
    int TCP_IP_ID = 0;

    uint32_t steerCnt = 0;
    // 判断是主驱动轮控制板还是辅助驱动轮控制板
    // 但主驱动轮和辅助驱动轮的一些参数数值不一样，需要在程序中区分装载

    void Init()
    {
        BaseType_t rtn;
        // 为指针指向的Class分配内存
        boardCmd_ = (MAIN_ASSIST_CMD *)pvPortMalloc(sizeof(MAIN_ASSIST_CMD));
        if (boardCmd_ == nullptr)
            return;
        boardVal_ = (MAIN_ASSIST_VAL *)pvPortMalloc(sizeof(MAIN_ASSIST_VAL));
        if (boardVal_ == nullptr)
            return;

        // Create tasks
        rtn = xTaskCreate(main_assist_board_task, (const portCHAR *)"steerTask",
                          tskStkSize, NULL, osPriorityAboveNormal, NULL);
        configASSERT(rtn == pdPASS);
    }

    void main_assist_board_task(void *pvParameters)
    {
        BaseType_t rtn;
        // * 舵轮控制需要的量
        uint8_t zero = 1;
        float dr2_delta_p = 0.f; // 舵电机的角度增量

        // * 控制机构电机需要的量（暂无）
        while(true)
        {
            // rtn = xSemaphoreTake(mainAssistTickSem, 2);
            // configASSERT(rtn);
            vTaskDelay(tskPeriod); // 5ms
            steerCnt++;

            // * 接收控制指令            
            // boardCmd_中包含了舵轮的目标速度和位置以及主控制板的夹紧状态和辅助控制板的夹角
            xQueueReceive(TskEth::mainAssistCmdQueue, boardCmd_, 0);
            // * 限定电机控制信息的更新频率，每5ms更新一次
            if(steerCnt % motorTick == 0)
            {
                // 接收到了重置信息，但当前舵轮的状态不是复位
                if(boardCmd_->state == steerState::STOP && boardVal_->state == steerState::STOP)
                {
                    // // 上电状态，失能所有电机
                    // usedMotors[0].ctrl_mode = PID_MODE::PID_MODE_IDLE; // 轮电机失能
                    // usedMotors[1].ctrl_mode = PID_MODE::PID_MODE_IDLE; // 舵电机失能
                    // usedMotors[2].ctrl_mode = PID_MODE::PID_MODE_IDLE; // 夹紧电机失能
                    boardVal_->state = steerState::STOP; // 停止状态
                }
                else if( boardCmd_->state == steerState::RESET && boardVal_->state != steerState::RESET)
                {
                    boardVal_->state = steerState::STOP;
                    usedMotors[0].set_tar(PID_MODE::PID_MODE_VELOCITY, 0.f); // 轮电机锁死
                    usedMotors[1].set_tar(PID_MODE::PID_MODE_VELOCITY, 0.f); // 舵电机停止旋转
                    // 此时轮电机与舵电机的速度都低，且当前控制状态不是复位中，所以就将当前终态置为复位中
                    // 也即先让舵轮停下来，然后开始进行复位流程
                    if (fabs(usedMotors[0].vel_current) < 0.05f && fabs(usedMotors[1].vel_current) < 0.05f)
                    boardVal_->state = steerState::RESET;
                }
                // 舵轮已经满足了复位要求，进入复位流程，开始进行
                else if(boardCmd_->state == steerState::RESET && boardVal_->state == steerState::RESET)
                {
                    // * 正式的复位流程，舵电机保持旋转，轮电机锁死，直到找到零点传感器位置
                    usedMotors[0].set_tar(PID_MODE::PID_MODE_VELOCITY, 0.f); // 轮电机锁死
                    usedMotors[1].set_tar(PID_MODE::PID_MODE_VELOCITY, - PI / 8.f); // 舵电机定速旋转
                    // 检测到了激光传感器的位置，发送复位信息
                    // if(GPIO_PIN_SET == HAL_GPIO_ReadPin(LaserSensor0_GPIO_Port, LaserSensor0_Pin))
                    // {
                    //     // 主动读取激光传感器GPIO，触发复位消息
                    //     xQueueSend(reset_flag, (void *)&zero, 0);
                    //     usedMotors[0].set_tar(PID_MODE::PID_MODE_VELOCITY, 0.f); // 轮电机锁死
                    //     usedMotors[1].set_tar(PID_MODE::PID_MODE_VELOCITY, 0.f); // 舵电机停止旋转
                    //     boardCmd_->state = steerState::STOP; // 复位完成，舵电机停止旋转
                    //     usedMotors[1].set_cali_val(caliAngel_sensor[TCP_IP_ID]); // 设置舵电机的标定值
                    // }else 
                    if(usedMotors[1].work_log == WORKING_LOG::BLOCK){
                        // 由于复位过程单向进行，所以可能会出现舵电机初始位置已经超过了激光传感器的位置
                        // 此时舵电机会一直旋转到限位位置保持堵转，需要读取电机的工作状态来发布另一种复位消息
                        xQueueSend(reset_flag, (void *)&zero, 0);
                        // usedMotors[0].set_tar(PID_MODE::PID_MODE_VELOCITY, 0.f); // 轮电机锁死
                        // usedMotors[1].set_tar(PID_MODE::PID_MODE_VELOCITY, 0.f); // 舵电机停止旋转
                        usedMotors[0].ctrl_mode = PID_MODE::PID_MODE_IDLE; // 轮电机失能
                        usedMotors[1].ctrl_mode = PID_MODE::PID_MODE_IDLE; // 舵电机失能    
                        usedMotors[1].set_cali_val(caliAngle_mech[TCP_IP_ID]); // 设置舵电机的标定值
                        
                        boardCmd_->state = steerState::STOP; // 复位完成，舵电机停止旋转
                        boardVal_->state = steerState::STOP; // 复位完成，舵电机停止旋转
                    }
                }
                else if(boardCmd_->state == steerState::NORMAL){
                    // 正常运行时进入这里
                    boardVal_->state = boardCmd_->state;
                    // dr1 限幅，单位换算 m/s to rad/s，计算增量
                    // 计算轮电机与期望速度的差值，并将偏差m/s转换为rpm
                    // 这里的dr1_tar_vel是电机转速环的输入值，配置轮电机速度
                    usedMotors[0].set_tar(PID_MODE::PID_MODE_VELOCITY, saturate(boardCmd_->dr1_tar_vel, maxVel, -maxVel)); 
                    //  配置舵电机角度
                    dr2_delta_p = cacul_ppi_angle(saturate(boardCmd_->dr2_tar_pos, PI, -PI), usedMotors[1].pos_current); // rad
                    usedMotors[1].set_delta(PID_MODE::PID_MODE_POSITION, dr2_delta_p); // 这里的dr2_tar_pos是电机位置环的输入值，配置舵电机角度
                }
                else if(boardCmd_->state == steerState::STOP){
                    usedMotors[0].resetVelPid();
                    usedMotors[1].resetPosPid();
                    usedMotors[1].resetVelPid();
                    boardVal_->state = steerState::STOP;
                    usedMotors[0].set_tar(PID_MODE::PID_MODE_VELOCITY, 0.f); // 轮电机锁死
                    usedMotors[1].set_tar(PID_MODE::PID_MODE_VELOCITY, 0.f); // 舵电机停止旋转
                }   
            }

            // ! 在这里插入第三个机构电机的控制代码
            // * 首先从传感器获取测量值
            // adcSensor->getSensorVal();
            HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcOri, 3); // 启动ADC转换
            if(boradType_ == BORAD_TYPE::MAIN_BOARD){
                // 主控制板有两个传感器，分别是左右弹簧的长度
            }else if(boradType_ == BORAD_TYPE::ASSIST_BOARD){
                int index = 0; // 辅助控制板有一个传感器 ， 夹角传感器参数索引
                // 前左： 0，对应参数数组0号索引 // 前右： 1，对应参数数组1号索引
                // 后左： 3，对应参数数组2号索引 // 后右： 4，对应参数数组3号索引
                index = TCP_IP_ID > 1 ? TCP_IP_ID - 1: TCP_IP_ID; // 夹角传感器参数索引
                adcVal[0] = float(adcOri[0]) * adc_angle_coeff[index] + adc_angle_offset[index]; // 夹角传感器的值
                usedMotors[2].pos_current = angle_stand_deg(adcVal[0]); // 夹角传感器的值
            }
            
            // ! 机构电机控制结束
            // todo 发送can控制帧、存储反馈值，并判断是否发送
            // boardVal_->dr1_real_vel = usedMotors[0].vel_current;
            // boardVal_->dr1_tar_vel  = usedMotors[0].vel_tar;
            // boardVal_->dr2_real_pos = usedMotors[1].pos_current;
            // boardVal_->dr2_tar_pos  = usedMotors[1].pos_tar;
            // boardVal_->dr2_real_vel = usedMotors[1].vel_current;
            // boardVal_->dr2_tar_vel  = usedMotors[1].vel_tar;
            // boardVal_->dr3_real_angle = usedMotors[2].pos_current;
            // boardVal_->dr3_tar_angle = usedMotors[2].pos_tar;
            // 为了测试发送缺省值
            boardVal_->state = boardCmd_->state;
            boardVal_->dr1_tar_vel  = 1.f;
            boardVal_->dr1_real_vel = 2.f;
            boardVal_->dr2_tar_vel  = 3.f;
            boardVal_->dr2_real_vel = 4.f;
            boardVal_->dr2_tar_pos  = 5.f;
            boardVal_->dr2_real_pos = 6.f;
            boardVal_->dr3_tar_angle = 7.f;
            boardVal_->dr3_real_angle = 8.f;
            boardVal_->dr3_tar_spring = 9.f;
            boardVal_->real_spring1 = 10.f;
            boardVal_->real_spring2 = 11.f;
            if(steerCnt % ethPeriod == 0)
            {
                // * 每隔ethPeriod个周期将舵轮的状态信息发送到以太网
                xQueueSend(TskEth::mainAssistValQueue, boardVal_, 0);
            }
        }
    }
};