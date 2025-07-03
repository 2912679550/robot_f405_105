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

float last_distance = 0.0f; // 用于构建舵轮里程计
float current_distance = 0.0f; // 用于构建舵轮里程计
float corr_axis = 0.0f; // 机器人在轴向前进的距离
float corr_cir  = 0.0f; // 机器人在周向移动的距离

namespace TskMainAssistBoard
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
            if(rtn == pdPASS){ pcOnline = true;} // 连接上PC的标志位
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
            // * 舵轮控制任务
            steer_control_sub_task();
            // * 里程计自刷新
            odom_sub_task();
            // * 根据当前的控制板类型，执行对应的机构电机驱动任务
            if(boradType_ == BORAD_TYPE::MAIN_BOARD) main_board_sub_tsk();
            else if (boradType_ == BORAD_TYPE::ASSIST_BOARD) assist_board_sub_tsk();
#endif

            // todo 发送can控制帧、存储反馈值，并判断是否发送
#if main_assist_debug
            // 为了测试发送缺省值
            boardVal_->state = boardCmd_->state;

#else
            // 机构电机的数据已经在子任务中完成装填
            boardVal_->dr1_real_vel = usedMotors[0].vel_current;    // 用于在上位机监控当前轮电机电流
            boardVal_->dr2_real_vel = usedMotors[1].vel_current;
            boardVal_->dr2_real_pos = usedMotors[1].pos_current;
#endif
            // * 每隔ethPeriod个周期将舵轮的状态信息发送到以太网
            xQueueSend(TskEth::mainAssistValQueue, boardVal_, 0);
        }
    }

    void odom_sub_task(){
        static bool firstFlag = true; // 用于判断是否是第一次执行里程计计算
        if( boardVal_ -> state == steerState::RESET){
            // 在舵轮执行复位流程时重置里程计
            last_distance = 0.0f; // 重置里程计
            current_distance = 0.0f; // 重置里程计
            corr_axis = 0.0f; // 重置轴向里程计
            corr_cir = 0.0f; // 重置周向里程计
            firstFlag = true; // 重置第一次执行标志位
        }else{
            if(firstFlag == true){
                firstFlag = false;
                current_distance = usedMotors[0].pos_current * usedMotors[0].mech_dir;
                last_distance = current_distance; // 初始化上一次的距离
                corr_axis = 0.0f; // 初始化轴向里程计
                corr_cir = 0.0f; // 初始化周向里程计
            }else{
                last_distance = current_distance; // 更新上一次的距离
                current_distance = usedMotors[0].pos_current * usedMotors[0].mech_dir; // 获取当前的距离
                float distance_diff = current_distance - last_distance; // 计算距离差
                corr_axis += distance_diff * sin(usedMotors[1].pos_current); // 轴向里程计，假设舵轮的转向角度为0时，机器人沿着轴向前进
                corr_cir  += distance_diff * cos(usedMotors[1].pos_current); // 周向里程计，假设舵轮的转向角度为0时，机器人沿着周向前进
            }
        }
        // 更新里程计值
        boardVal_->odom_axis = corr_axis; // 机器人在轴向前进的距离
        boardVal_->odom_cir = corr_cir; // 机器人在周向移动的距离
    }

    void steer_control_sub_task(){
        static uint8_t zero = 1;
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
                    // usedMotors[0].set_tar(PID_MODE::PID_MODE_VELOCITY, 0.f); // 轮电机锁死
                    // usedMotors[1].set_tar(PID_MODE::PID_MODE_VELOCITY, 0.f); // 舵电机停止旋转
                    usedMotors[0].set_tar(PID_MODE::PID_MODE_VELOCITY, 0.f); // 轮电机锁死
                    usedMotors[1].set_tar(PID_MODE::PID_MODE_POSITION, 0.5 * PI); // 舵电机标定完成后转到90°的位置
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
    }

    void main_board_sub_tsk(){
        static float dr3_tar_spring = 0.0f;
        static bool spring_length_arrived = false; // 用于判断弹簧长度是否到达目标值
        HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcOri, 3); // 启动ADC转换
        int index = 0;
        // 前中： 2 ， 对应参数数组0号索引
        // 后中： 5 ， 对应参数数组1号索引
        index = TCP_IP_ID < 3 ? 0 : 1; // 弹簧传感器参数索引
        adcVal[0] = float(adcOri[0]) * adc_spring_coeff[index][0] + adc_spring_offset[index][0]; // 弹簧传感器的值
        adcVal[1] = float(adcOri[1]) * adc_spring_coeff[index][1] + adc_spring_offset[index][1]; // 弹簧传感器的值
        usedMotors[2].pos_current = (adcVal[0] + adcVal[1]) / 2.0f; // 弹簧传感器的值,取平均值
        // 中间丝杠电机的控制逻辑
        dr3_tar_spring = boardCmd_->dr3_tar_tight;
        boardVal_->real_spring1 = adcVal[0];    // 存储左右测距传感器测量得到的弹簧长度
        boardVal_->real_spring2 = adcVal[1]; 
        float mean_spring_length = (adcVal[0] + adcVal[1]) / 2.0f; // 平均弹簧长度

        // todo 250703 控制逻辑2： 将丝杆弹簧压缩量改为连续可调模式
        dr3_tar_spring = saturate(dr3_tar_spring, spring_length_limit[1], spring_length_limit[0]); // 限制弹簧长度的范围

        if( abs(mean_spring_length - dr3_tar_spring) < 0.5f){
            spring_length_arrived = true;
            usedMotors[2].ctrl_mode = PID_MODE::PID_MODE_IDLE;  // 到达期望位置，电机失能
            return; // 直接返回
        }else{
            // 当前长度与期望值之间存在差异
            spring_length_arrived = false; // 弹簧长度未到达目标值
            if(mean_spring_length >= dr3_tar_spring){   // 弹簧长度大于等于目标值，表示希望夹紧
                usedMotors[2].set_tar(PID_MODE::PID_MODE_VELOCITY, 15.0f); // 夹紧
                if(
                    usedMotors[2].work_log == WORKING_LOG::BLOCK ||
                    (
                        // 两遍的弹簧长度都小于等与目标值，说明已经被压缩到位
                        adcVal[0] <= dr3_tar_spring &&
                        adcVal[1] <= dr3_tar_spring
                    )
                ){
                    springTight = true; // 给出夹紧成功的标志，相比于原来，这里就不再存储夹紧长度了
                }
            }else{
                if(HAL_GPIO_ReadPin(LaserSensor0_GPIO_Port, LaserSensor0_Pin) == 1){
                    // 触发到了电子限位器，此时就不能继续让电机再松开了
                    usedMotors[2].ctrl_mode = PID_MODE::PID_MODE_IDLE;  // 到达期望位置，电机失能
                    return; // 直接返回
                }else{
                    usedMotors[2].set_tar(PID_MODE::PID_MODE_VELOCITY, -15.0f); // 松开
                }
            }
        }
        // todo 控制逻辑2 完结

        // todo： 控制逻辑1：夹紧丝杠电机跑速度环，若tar大于10，表示希望夹紧，反之表示希望松开
        // if( dr3_tar_spring > 10.0f){
        //     /*
        //     // 期望夹紧 , 夹紧的成功仅通过电机是否堵转来判断，并给一个较大的判断阈值
        //     // 夹紧成功就将电机失能，直到检测到弹簧的压缩量超出了阈值，再重新给夹紧失效，使得电机重新开始工作直到堵转
        //     // todo： 老版的通过电机堵转电流来判断夹紧是否成功的程序
        //     // if( spring_tight_length_[0] - adcVal[0] < 10.0f && 
        //     //     spring_tight_length_[1] - adcVal[1] < 10.0f &&
        //     //     springTight == true){
        //     // }else{
        //     //     springTight = false; // 夹紧状态失效，重新开始夹紧
        //     // }
        //     // // 执行
        //     // if(springTight == false){
        //     //     usedMotors[2].set_tar(PID_MODE::PID_MODE_VELOCITY, 15.0f); // 夹紧
        //     //     if(
        //     //         // 夹紧电机堵转或者到达传感器阈值范围之内
        //     //         usedMotors[2].work_log == WORKING_LOG::BLOCK ||
        //     //         (
        //     //             adcVal[0] >= adc_spring_val[index][0] && 
        //     //             adcVal[1] >= adc_spring_val[index][1]
        //     //         )
        //     //     ){
        //     //         springTight = true;
        //     //         // 存储当前加紧值
        //     //         spring_tight_length_[0] = adcVal[0]; // 左右测距传感器测量得到的弹簧长度
        //     //         spring_tight_length_[1] = adcVal[1]; // 左右测距传感器测量得到的弹簧长度
        //     //     }
        //     // }else{
        //     //     usedMotors[2].ctrl_mode = PID_MODE::PID_MODE_IDLE; // 夹紧完成，电机失能
        //     // }
        //     */
        //     // TODO： 新版的通过直接测量弹簧长度来控制夹紧过程
        //     dr3_tar_spring = saturate( dr3_tar_spring , spring_length_limit[1], spring_length_limit[0]); // 限制弹簧长度的范围
        //     // * 这部分逻辑用来判断夹紧成功后是否夹紧失效了
        //     if( (adcVal[0] - dr3_tar_spring > 1.0 ||
        //          adcVal[1] - dr3_tar_spring > 1.0) &&
        //         springTight == true){
        //         // 夹紧状态失效，重新开始夹紧
        //         springTight = false; // 夹紧状态失效，重新开始夹紧
        //     }
        //     if(springTight == false){
        //         usedMotors[2].set_tar(PID_MODE::PID_MODE_VELOCITY, 15.0f); // 夹紧
        //         if(
        //             usedMotors[2].work_log == WORKING_LOG::BLOCK ||
        //             (
        //                 // 两遍的弹簧长度都小于等与目标值，说明已经被压缩到位
        //                 adcVal[0] <= dr3_tar_spring &&
        //                 adcVal[1] <= dr3_tar_spring
        //             )
        //         ){
        //             springTight = true; // 给出夹紧成功的标志，相比于原来，这里就不再存储夹紧长度了
        //         }
        //     }else{
        //          usedMotors[2].ctrl_mode = PID_MODE::PID_MODE_IDLE; // 夹紧完成，电机失能
        //     }
        // }else{
        //     // 期望松开，松开的指标设计为弹簧的长度达到最大值或者出发了限位传感器报警
        //     debug = HAL_GPIO_ReadPin(LaserSensor0_GPIO_Port, LaserSensor0_Pin);
        //     if (debug == 1)
        //     {
        //         // 触发松开完成标志，失能电机
        //         // debug 对应的传感器在有金属杆时为0，没金属杆时为1
        //         usedMotors[2].ctrl_mode = PID_MODE::PID_MODE_IDLE;  // 夹紧完成，电机失能
        //         springTight = false; // 夹紧状态失效
        //     }else{
        //         // 没有触发松开完成标志
        //         usedMotors[2].set_tar(PID_MODE::PID_MODE_VELOCITY, -15.0f); // 松开
        //     }
        // }
        // todo 控制逻辑1 完结

    }

    void assist_board_sub_tsk(){
        static float dr3_tar_angle = 0.0f; // 夹角电机的目标角度
        // * 首先是按照传感器信息获取当前电机的位置值
        HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcOri, 3); // 启动ADC转换
        int index = 0; // 辅助控制板有一个传感器 ， 夹角传感器参数索引
        // 前左： 0，对应参数数组0号索引 // 前右： 1，对应参数数组1号索引
        // 后左： 3，对应参数数组2号索引 // 后右： 4，对应参数数组3号索引
        index = TCP_IP_ID > 1 ? TCP_IP_ID - 1: TCP_IP_ID; // 夹角传感器参数索引
        adcVal[0] = float(adcOri[0]) * adc_angle_coeff[index] + adc_angle_offset[index]; // 夹角传感器的值
        usedMotors[2].pos_current = angle_stand_deg(adcVal[0]); // 夹角传感器的值
        // * 存储反馈值并控制电机
        dr3_tar_angle = saturate(boardCmd_->dr3_tar_angle, mechAngleRange[1], mechAngleRange[0]); // 夹角传感器的值
        boardVal_->dr3_real_angle = usedMotors[2].pos_current; // 夹角传感器的值
        // 夹角电机的控制逻辑
        if(boardCmd_->dr3_tar_tight > 10.0f){
            // 发来夹紧指令时就不在控制夹角电机
            usedMotors[2].ctrl_mode = PID_MODE::PID_MODE_IDLE;
        }else{
            usedMotors[2].set_tar(PID_MODE::PID_MODE_POSITION, dr3_tar_angle); // 夹角电机位置环的输入值，配置舵电机角度
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