#include "main_assist_board_tsk.h"
#include "ethernet_tsk.h"
#include "math.h"
#include "motor.h"
#include "adcSensor.h"
#include "public_func.h"

#define main_assist_debug 0

uint16_t adcOri[3] = {0};
float adcVal[3] = {0};
uint16_t debug = 0;
bool springTight = false;
bool pcOnline = false; // 连接上PC的标志位

namespace TskSteerBoard
{
    const int tskStkSize = 512;
    // * 控制指令
    uint8_t tskPeriod = 5; // ms
    MAIN_ASSIST_CMD *boardCmd_ = nullptr; // 用于控制辅助驱动轮的命令
    MAIN_ASSIST_VAL *boardVal_ = nullptr; // 用于反馈辅助驱动轮的状态
    // 实测发现压缩量传感器的数据并不稳定，所以每次夹紧后存储一下当前的夹紧长度，并在后续通过当前的夹紧长度来判断加紧是否失效
    float spring_tight_length_[2] = {0.0f, 0.0f}; 

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
        // 给一个命令的初始值
        boardCmd_->state = steerState::STOP;
        boardCmd_->dr1_tar_vel = 0.f; // 舵轮的目标速度
        boardCmd_->dr2_tar_pos = 0.5f; // 舵轮的目标位置
        boardCmd_->dr3_tar_angle = mechAngleRange[0];
        boardCmd_->dr3_tar_tight = 0.f; // 夹紧状态，默认松开
        boardVal_ = (MAIN_ASSIST_VAL *)pvPortMalloc(sizeof(MAIN_ASSIST_VAL));
        if (boardVal_ == nullptr)
            return;

        load_settings_();

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
            // 测试传感器
            debug = HAL_GPIO_ReadPin(LaserSensor0_GPIO_Port, LaserSensor0_Pin);
            // rtn = xSemaphoreTake(mainAssistTickSem, 2);
            // configASSERT(rtn);
            // * 限定电机控制信息的更新频率，每5ms更新一次
            vTaskDelay(tskPeriod); // 5ms
            steerCnt++;

            // * 接收控制指令
            BaseType_t rtn;
            // boardCmd_中包含了舵轮的目标速度和位置以及主控制板的夹紧状态和辅助控制板的夹角
            rtn = xQueueReceive(TskEth::mainAssistCmdQueue, boardCmd_, 0);
            if(rtn == pdPASS){
                pcOnline = true; // 连接上PC的标志位
            }
        #if main_assist_debug
            // * 首先从传感器获取测量值
            // adcSensor->getSensorVal();
            HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcOri, 3); // 启动ADC转换
            if(boradType_ == BORAD_TYPE::MAIN_BOARD){
                // 主控制板有两个传感器，分别是左右弹簧的长度
                int index = 0;
                // 前中： 2 ， 对应参数数组0号索引
                // 后中： 5 ， 对应参数数组1号索引
                index = TCP_IP_ID < 3 ? 0 : 1; // 弹簧传感器参数索引
                adcVal[0] = float(adcOri[0]) * adc_spring_coeff[index][0] + adc_spring_offset[index][0]; // 弹簧传感器的值
                adcVal[1] = float(adcOri[1]) * adc_spring_coeff[index][1] + adc_spring_offset[index][1]; // 弹簧传感器的值
                usedMotors[2].pos_current = (adcVal[0] + adcVal[1]) / 2.0f; // 弹簧传感器的值,取平均值
                // 中间丝杠电机的控制逻辑
            }else if(boradType_ == BORAD_TYPE::ASSIST_BOARD){
                int index = 0; // 辅助控制板有一个传感器 ， 夹角传感器参数索引
                // 前左： 0，对应参数数组0号索引 // 前右： 1，对应参数数组1号索引
                // 后左： 3，对应参数数组2号索引 // 后右： 4，对应参数数组3号索引
                index = TCP_IP_ID > 1 ? TCP_IP_ID - 1: TCP_IP_ID; // 夹角传感器参数索引
                adcVal[0] = float(adcOri[0]) * adc_angle_coeff[index] + adc_angle_offset[index]; // 夹角传感器的值
                usedMotors[2].pos_current = angle_stand_deg(adcVal[0]); // 夹角传感器的值
            }
        #else
            if(pcOnline == false) continue;    
        // ! 舵轮状态机控制逻辑
            switch(boardCmd_->state){
                case steerState::RESET:
                    // 接收到了重置信息，但当前舵轮的状态不是复位或舵轮有速度
                    if(boardVal_->state < steerState::RESET)
                    {
                        boardVal_->state = steerState::STOP;    // 将实际状态机调整为stop，并给电机发布停转指令
                        usedMotors[0].set_tar(PID_MODE::PID_MODE_VELOCITY, 0.f); // 轮电机锁死
                        usedMotors[1].set_tar(PID_MODE::PID_MODE_VELOCITY, 0.f); // 舵电机停止旋转
                        // 此时轮电机与舵电机的速度都低，且当前控制状态不是复位中，所以就将当前终态置为复位中
                        // 也即先让舵轮停下来，然后开始进行复位流程
                        if (fabs(usedMotors[0].vel_current) < 0.05f && fabs(usedMotors[1].vel_current) < 0.05f)
                        boardVal_->state = steerState::RESET;
                    }
                    // 舵轮已经满足了复位要求，可以进入复位流程
                    else if(boardVal_->state == steerState::RESET)
                    {
                        // * 正式的复位流程，舵电机保持旋转，轮电机锁死，直到找到零点传感器位置
                        usedMotors[0].set_tar(PID_MODE::PID_MODE_VELOCITY, 0.f); // 轮电机锁死
                        usedMotors[1].set_tar(PID_MODE::PID_MODE_VELOCITY, - PI / 8.f); // 舵电机定速旋转
                        if(usedMotors[1].work_log == WORKING_LOG::BLOCK){
                            // 由于复位过程单向进行，所以可能会出现舵电机初始位置已经超过了激光传感器的位置
                            // 此时舵电机会一直旋转到限位位置保持堵转，需要读取电机的工作状态来发布另一种复位消息
                            xQueueSend(reset_flag, (void *)&zero, 0);
                            usedMotors[0].ctrl_mode = PID_MODE::PID_MODE_IDLE; // 轮电机失能
                            usedMotors[1].ctrl_mode = PID_MODE::PID_MODE_IDLE; // 舵电机失能    
                            usedMotors[1].set_cali_val(caliAngle_mech[TCP_IP_ID]); // 设置舵电机的标定值
                            boardVal_->state = steerState::RESET_OVER; // 复位完成
                        }
                    }
                    else if(boardVal_ ->state == steerState::RESET_OVER){
                        // 复位完成，舵电机停止旋转，轮电机锁死，等待新的命令，且此时不能再接收复位命令
                        usedMotors[0].set_tar(PID_MODE::PID_MODE_VELOCITY, 0.f); // 轮电机锁死
                        usedMotors[1].set_tar(PID_MODE::PID_MODE_VELOCITY, 0.f); // 舵电机停止旋转
                    }
                    break;
                case steerState::NORMAL:
                    // 接收到了正常的运行指令
                    if(usedMotors[1].cali_flag == true){
                        // 只有经过标定后，机器人才响应接收到的舵轮方向与舵轮速度指令
                        boardVal_->state = steerState::NORMAL;
                        // * 舵电机目标值限位
                        usedMotors[1].set_tar(PID_MODE::PID_MODE_POSITION, saturate(boardCmd_->dr2_tar_pos, steerDirRange[1], steerDirRange[0])); // 舵电机位置环的输入值，配置舵电机角度
                        if( ABSF(usedMotors[1].pos_current - boardCmd_->dr2_tar_pos) < 0.1f ){
                            // 此时舵电机接近到位，可以开启轮电机
                            usedMotors[0].set_tar(PID_MODE::PID_MODE_VELOCITY, saturate(boardCmd_->dr1_tar_vel, maxVel, -maxVel)); 
                        }else{
                            usedMotors[0].set_tar(PID_MODE::PID_MODE_VELOCITY, 0.f); // 轮电机锁死
                        }
                    }
                    break;
                case steerState::STOP:
                    // 任何情况下都可以执行STOP状态，包括复位时
                    boardVal_ -> state = steerState::STOP;
                    usedMotors[0].set_tar(PID_MODE::PID_MODE_VELOCITY, 0.f); // 轮电机锁死
                    usedMotors[1].set_tar(PID_MODE::PID_MODE_VELOCITY, 0.f); // 舵电机锁死
                    break;
                default:
                    break;
            }

            if(boradType_ == BORAD_TYPE::MAIN_BOARD){
                main_board_sub_tsk();
            }else if (boradType_ == BORAD_TYPE::ASSIST_BOARD){
                assist_board_sub_tsk();
            }
        #endif

            // todo 发送can控制帧、存储反馈值，并判断是否发送
        #if main_assist_debug
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
        #else
            boardVal_->dr1_real_vel = usedMotors[0].vel_current;
            boardVal_->dr1_tar_vel  = usedMotors[0].vel_tar;
            boardVal_->dr2_real_pos = usedMotors[1].pos_current;
            boardVal_->dr2_tar_pos  = usedMotors[1].pos_tar;
            boardVal_->dr2_tar_vel = usedMotors[1].vel_tar;
            boardVal_->dr2_real_vel = usedMotors[1].vel_current;
            // 机构电机的数据已经在子任务中完成装填
        #endif
            // * 每隔ethPeriod个周期将舵轮的状态信息发送到以太网
            xQueueSend(TskEth::mainAssistValQueue, boardVal_, 0);
        }
    }


    void main_board_sub_tsk(){
        HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcOri, 3); // 启动ADC转换
        int index = 0;
        // 前中： 2 ， 对应参数数组0号索引
        // 后中： 5 ， 对应参数数组1号索引
        index = TCP_IP_ID < 3 ? 0 : 1; // 弹簧传感器参数索引
        adcVal[0] = float(adcOri[0]) * adc_spring_coeff[index][0] + adc_spring_offset[index][0]; // 弹簧传感器的值
        adcVal[1] = float(adcOri[1]) * adc_spring_coeff[index][1] + adc_spring_offset[index][1]; // 弹簧传感器的值
        usedMotors[2].pos_current = (adcVal[0] + adcVal[1]) / 2.0f; // 弹簧传感器的值,取平均值
        // 中间丝杠电机的控制逻辑
        boardVal_->dr3_tar_spring = boardCmd_->dr3_tar_tight;
        boardVal_->real_spring1 = adcVal[0];    // 左右测距传感器测量得到的弹簧长度
        boardVal_->real_spring2 = adcVal[1]; 

        // todo： 控制逻辑1：夹紧丝杠电机跑速度环，若tar大于10，表示希望夹紧，反之表示希望松开
        if(boardVal_->dr3_tar_spring > 10.0f){
            // 期望夹紧 , 夹紧的成功仅通过电机是否堵转来判断，并给一个较大的判断阈值
            // 夹紧成功就将电机失能，直到检测到弹簧的压缩量超出了阈值，再重新给夹紧失效，使得电机重新开始工作直到堵转
            if( spring_tight_length_[0] - adcVal[0] < 10.0f && 
                spring_tight_length_[1] - adcVal[1] < 10.0f &&
                springTight == true){

            }else{
                springTight = false; // 夹紧状态失效，重新开始夹紧
            }
            // 执行
            if(springTight == false){
                usedMotors[2].set_tar(PID_MODE::PID_MODE_VELOCITY, 15.0f); // 夹紧
                if(
                    // 夹紧电机堵转或者到达传感器阈值范围之内
                    usedMotors[2].work_log == WORKING_LOG::BLOCK ||
                    (
                        adcVal[0] >= adc_spring_val[index][0] && 
                        adcVal[1] >= adc_spring_val[index][1]
                    )
                ){
                    springTight = true;
                    // 存储当前加紧值
                    spring_tight_length_[0] = adcVal[0]; // 左右测距传感器测量得到的弹簧长度
                    spring_tight_length_[1] = adcVal[1]; // 左右测距传感器测量得到的弹簧长度
                }
            }else{
                usedMotors[2].ctrl_mode = PID_MODE::PID_MODE_IDLE; // 夹紧完成，电机失能
            }
        }else{
            // 期望松开，松开的指标设计为弹簧的长度达到最大值或者出发了限位传感器报警
            usedMotors[2].set_tar(PID_MODE::PID_MODE_VELOCITY, -15.0f); // 松开 15为测试的速度
            debug = HAL_GPIO_ReadPin(LaserSensor0_GPIO_Port, LaserSensor0_Pin);
            if (debug == 1)
            {
                // 触发松开完成标志，失能电机
                // debug 对应的传感器在有金属杆时为0，没金属杆时为1
                usedMotors[2].ctrl_mode = PID_MODE::PID_MODE_IDLE; // 夹紧完成，电机失能
            }
        }

    }
 
    void assist_board_sub_tsk(){
        // * 首先是按照传感器信息获取当前电机的位置值
        HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcOri, 3); // 启动ADC转换
        int index = 0; // 辅助控制板有一个传感器 ， 夹角传感器参数索引
        // 前左： 0，对应参数数组0号索引 // 前右： 1，对应参数数组1号索引
        // 后左： 3，对应参数数组2号索引 // 后右： 4，对应参数数组3号索引
        index = TCP_IP_ID > 1 ? TCP_IP_ID - 1: TCP_IP_ID; // 夹角传感器参数索引
        adcVal[0] = float(adcOri[0]) * adc_angle_coeff[index] + adc_angle_offset[index]; // 夹角传感器的值
        usedMotors[2].pos_current = angle_stand_deg(adcVal[0]); // 夹角传感器的值
        // * 存储反馈值并控制电机
        boardVal_->dr3_tar_angle = saturate(boardCmd_->dr3_tar_angle, mechAngleRange[1], mechAngleRange[0]); // 夹角传感器的值
        boardVal_->dr3_real_angle = usedMotors[2].pos_current; // 夹角传感器的值
        // 夹角电机的控制逻辑
        if(boardCmd_->dr3_tar_tight > 10.0f){
            // 发来夹紧指令时就不在控制夹角电机
            usedMotors[2].ctrl_mode = PID_MODE::PID_MODE_IDLE;
        }else{
            usedMotors[2].set_tar(PID_MODE::PID_MODE_POSITION, boardVal_->dr3_tar_angle); // 夹角电机位置环的输入值，配置舵电机角度
        }
    }

    void load_settings_(){
        if(boradType_ == BORAD_TYPE::MAIN_BOARD){
            int index = 0;
            // 前中： 2 ， 对应参数数组0号索引
            // 后中： 5 ， 对应参数数组1号索引
            index = TCP_IP_ID < 3 ? 0 : 1; // 弹簧传感器参数索引
            spring_tight_length_[0] = adc_spring_val[index][0];
            spring_tight_length_[1] = adc_spring_val[index][1]; // 弹簧传感器的值,取平均值
        }
    }
};