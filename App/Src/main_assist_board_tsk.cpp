#include "main_assist_board_tsk.h"
#include "ethernet_tsk.h"
#include "math.h"
#include "motor.h"

namespace TskSteerBoard
{
    const int tskStkSize = 512;
    // * 控制指令
    MAIN_ASSIST_CMD *boardCmd_ = nullptr; // 用于控制辅助驱动轮的命令
    MAIN_ASSIST_VAL *boardVal_ = nullptr; // 用于反馈辅助驱动轮的状态

    // * PID参数
    PID_PARAM* dr1VelPID_ = nullptr;      // 舵轮轮电机，跑速度环
    FFPID_PARAM* dr2PosPID_ = nullptr;    // 舵轮转向电机，跑位置环
    PID_PARAM* dr2VelPID_ = nullptr;
    FFPID_PARAM* dr3PosPID_ = nullptr;    // 用于控制机构电机的前置PID，用于处理传感器测量位置数据到电机期望速度的转换
    PID_PARAM* dr3VelPID_ = nullptr;      // 用于执行机构电机的速度环PID控制
    // * 结构参数
    float wheelRatio_ = 0.f;    // 轮电机电机的减速比
    float steerRatio_ = 0.f;    // 转向电机的减速比
    float wheelR_ = 0.f;        // 舵轮轮胎半径
    // * 信息容器
    moto_measure_t *motor_ = nullptr;
    BORAD_TYPE boradType_ = BORAD_TYPE::idle; // 控制板类型(这个任务中只会设置为主驱动轮或辅助驱动轮)
    // 用于存储ADC采样值，结合main文件中对ADC通道的配置：
    // PC0：ADC1_IN10  rank 1
    // PC1：ADC1_IN11  rank 2
    // PC2：ADC1_IN12  rank 3
    uint16_t adcVal[2] = {0}; 
    float adcRealVal[2] = {0}; // 用于存储经过变换后得到的ADC采样值，这时就已经与实际值对应了

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
        motor_ = (moto_measure_t *)pvPortMalloc(sizeof(moto_measure_t));
        if (motor_ == nullptr)
            return;
        load_params_(); // 读取参数
        load_motor_params_(); // 装载电机类的机械参数

        // Create tasks
        rtn = xTaskCreate(main_assist_board_task, (const portCHAR *)"steerTask",
                          tskStkSize, NULL, osPriorityAboveNormal, NULL);
        configASSERT(rtn == pdPASS);
    }

    void load_params_(){
        if(boradType_ == BORAD_TYPE::idle){
            return;
        }
        // TODO 这两个板子都需要完整的舵轮参数
        // 首先初始化指针空间
        dr1VelPID_ = (PID_PARAM *)pvPortMalloc(sizeof(PID_PARAM));
        dr2PosPID_ = (FFPID_PARAM *)pvPortMalloc(sizeof(FFPID_PARAM));
        dr2VelPID_ = (PID_PARAM *)pvPortMalloc(sizeof(PID_PARAM));
        // 按照控制板类型直接读读取在数组中的参数
        int index = boradType_ - 1;
        // 结构参数
        wheelRatio_ = wheelRatio[index];
        steerRatio_ = steerRatio[index];
        wheelR_ = wheelR[index];
        // dr1 vel loop
        dr1VelPID_->p = vVelPidP[index];
        dr1VelPID_->i = vVelPidI[index];
        dr1VelPID_->d = vVelPidD[index];
        dr1VelPID_->n = 1;
        dr1VelPID_->ts = motorTs;
        dr1VelPID_->outIMin = -0.95f * C610Current;
        dr1VelPID_->outIMax = 0.95f * C610Current;
        dr1VelPID_->Iband = vVelPidIband[index];
        dr1VelPID_->outMin = -0.95f * C610Current;
        dr1VelPID_->outMax = 0.95f * C610Current;
        // dr2 pos loop
        dr2PosPID_->p = thPosPidP[index];
        dr2PosPID_->i = thPosPidI[index];
        dr2PosPID_->wc = 50;
        dr2PosPID_->ts = motorTs;
        dr2PosPID_->outIMin = -maxSteerOmg / 5;
        dr2PosPID_->outIMax = maxSteerOmg / 5;
        dr2PosPID_->Iband = thPosPidIband[index];
        dr2PosPID_->outMin = -maxSteerOmg;
        dr2PosPID_->outMax = maxSteerOmg;
        // dr2 vel loop
        dr2VelPID_->p = thVelPidP[index];
        dr2VelPID_->i = thVelPidI[index];
        dr2VelPID_->d = thVelPidD[index];
        dr2VelPID_->n = 1;
        dr2VelPID_->ts = motorTs;
        dr2VelPID_->outIMin = -0.95f * C610Current;
        dr2VelPID_->outIMax = 0.95f * C610Current;
        dr2VelPID_->Iband = thVelPidIband[index];
        dr2VelPID_->outMin = -0.95f * C610Current;
        dr2VelPID_->outMax = 0.95f * C610Current;

        // * 装载到电机类中
        usedMotors[0].initMotorPid(dr1VelPID_, nullptr); // dr1 速度环
        usedMotors[1].initMotorPid(dr2VelPID_, dr2PosPID_); // dr2 速度环和位置环
    
        // TODO 主控板的机构电机只跑速度环，辅助驱动轮的机构电机跑位置环和速度环
        // 配置公用速度环参数
        dr3VelPID_ = (PID_PARAM *)pvPortMalloc(sizeof(PID_PARAM));
        dr3VelPID_->p = mechVelPidP[index];
        dr3VelPID_->i = mechVelPidI[index];
        dr3VelPID_->d = mechVelPidD[index];
        dr3VelPID_->n = 1;
        dr3VelPID_->ts = motorTs;
        dr3VelPID_->outIMin = -0.95f * C610Current;
        dr3VelPID_->outIMax = 0.95f * C610Current;
        dr3VelPID_->Iband = mechVelPidIband[index];
        dr3VelPID_->outMin = -0.95f * C610Current;
        dr3VelPID_->outMax = 0.95f * C610Current;
        // 辅助轮控制板需要额外配置一个位置环参数
        if(boradType_ == BORAD_TYPE::ASSIST_BOARD){
            dr3PosPID_ = (FFPID_PARAM *)pvPortMalloc(sizeof(FFPID_PARAM));
            dr3PosPID_->p = mechPosPidP[index];
            dr3PosPID_->i = mechPosPidI[index];
            dr3PosPID_->wc = 50;
            dr3PosPID_->ts = motorTs;
            dr3PosPID_->outIMin = -maxSteerOmg / 5;
            dr3PosPID_->outIMax = maxSteerOmg / 5;
            dr3PosPID_->Iband = mechPosPidIband[index];
            dr3PosPID_->outMin = -maxSteerOmg;
            dr3PosPID_->outMax = maxSteerOmg;
        }

        // * 装载到电机类中
        usedMotors[2].initMotorPid(dr3VelPID_, dr3PosPID_); // dr3 速度环和位置环
    }

    void load_motor_params_(){
        usedMotors[0].can2cur = C610ICoeff; // dr1 轮电机
        usedMotors[0].can2vel = wheelR_ / wheelRatio_ / toRPM; // dr1 轮电机,表示的是轮子的线速度
        usedMotors[0].can2pos = 1 / wheelRatio_ / 8192.0f * 2.f * PI; // dr1 轮电机,单位为rad

        usedMotors[1].can2cur = C610ICoeff; // dr2 转向电机
        usedMotors[1].can2vel = 1 / steerRatio_ / toRPM; // dr2 转向电机，单位为rad/s
        usedMotors[1].can2pos = 1 / steerRatio_ / 8192.0f * 2.f * PI; // dr2 转向电机,单位为rad

        // * 暂时保留
        usedMotors[2].can2cur = C610ICoeff; // dr3 机构电机
        usedMotors[2].can2vel = 1; // dr3 机构电机，单位为rad/s
        usedMotors[2].can2pos = 0; // dr3 机构电机,单位为rad
    }

    void io_config_(){
        // todo 主驱动轮控制板需要两个读取电平的IO口，还需要两个读取电压的ADC口

        // todo 辅助驱动轮只需要一个读取电压的ADC口
    }

    void main_assist_board_task(void *pvParameters)
    {
        BaseType_t rtn;
        can1RxQueueHandle = xQueueCreate(4, sizeof(moto_measure_t));
        // 初始化CAN并使能中断
        CAN_Start_Trans();

        // * 舵轮控制需要的量
        float dr1_delta_v = 0.f, dr2_delta_p = 0.f;
        float dr1_tar_i = 0.f, dr2_tar_i = 0.f;
        uint8_t zero = 1;
        uint8_t curCmd[8] = {0};
        // 上电时电流均给0
        CAN_SendMsg(CAN_Moto_ALL_ID, curCmd);

        // * 控制机构电机需要的量（暂无）

        while(true)
        {
            rtn = xSemaphoreTake(mainAssistTickSem, 2);
            configASSERT(rtn);
            steerCnt++;

            // * 接收电机与传感器状态数据
            if (pdPASS == xQueueReceive(can1RxQueueHandle, motor_, 0) && motor_ -> id < 3)
                usedMotors[motor_->id].unpackCanData(motor_);   // 此时已经将can帧的消息解包到电机类中
            HAL_ADC_Start_DMA(&hadc1 , (uint32_t *)adcVal, 2); // 启动ADC采样，读取电压值
            
            // 控制信息接收，boardCmd_中包含了舵轮的目标速度和位置以及主控制板的夹紧状态和辅助控制板的夹角
            xQueueReceive(TskEth::mainAssistCmdQueue, boardCmd_, 0);
            // * 限定电机控制信息的更新频率，每5ms更新一次
            if(steerCnt % motorTick == 0)
            {
                if( boardCmd_->state == steerState::RESET && boardVal_->state != steerState::RESETOVER)
                {
                    if (boardVal_->state != steerState::RESETTING && 
                        fabs(usedMotors[0].vel_current) < 0.05f && 
                        fabs(usedMotors[1].vel_current) < 0.05f)
                    {
                        // 此时轮电机与舵电机的速度都低，且当前控制状态不是复位中，所以就将当前终态置为复位中
                        // boardCmd_->dr1_tar_vel = 0.f;
                        // pi/2 rad/s旋转
                        // boardCmd_->dr2_tar_pos = 0.f;
                        boardVal_->state = steerState::RESETTING;
                        usedMotors[0].vel_tar = 0.f;   // 此时保持轮电机不转，舵电机定速旋转
                        usedMotors[1].pos_tar = 0.f; // 
                        usedMotors[1].vel_tar = 0.f;
                        // boardVal_->dr2_tar_vel = 0.f;
                    }
                    else if (boardVal_->state == steerState::RESETTING)
                    {
                        // 此时正在执行复位中，目的是保持轮电机锁死，并旋转舵电机直到找到传感器电平变化的位置
                        // pi/2 rad/s旋转
                        // boardVal_->dr2_tar_vel = PI / 4.f;
                        usedMotors[0].vel_tar = 0.f;   // 此时保持轮电机不转，舵电机定速旋转
                        usedMotors[1].pos_tar = 0.f;
                        usedMotors[1].vel_tar = PI / 4.f;   // 此时保持轮电机不转，舵电机定速旋转
                    }
                    else{
                        // 当舵轮正在运行时，先让其进入stop状态
                        eprint((char *)"MUST stop before resetting");
                        boardVal_->state = steerState::STOP;
                    }
                    
                    if (GPIO_PIN_SET == HAL_GPIO_ReadPin(LaserSensor0_GPIO_Port, LaserSensor0_Pin))
                    {
                        // 主动读取激光传感器GPIO，触发复位消息
                        xQueueSend(reset_flag, (void *)&zero, 0);
                        boardVal_->state = steerState::RESETOVER;
                    }
                }
                else if(boardCmd_->state == steerState::NORMAL){
                    // 正常运行时进入这里
                    boardVal_->state = boardCmd_->state;
                    // dr1 限幅，单位换算 m/s to rad/s，计算增量
                    // 计算轮电机与期望速度的差值，并将偏差m/s转换为rpm
                    dr1_delta_v = saturate(boardCmd_->dr1_tar_vel, maxVel, -maxVel) - usedMotors[0].vel_current;    // uint:m/s
                    dr2_delta_p = cacul_ppi_angle(saturate(boardCmd_->dr2_tar_pos, PI, -PI), usedMotors[1].pos_current); // rad
                }
                else if(boardCmd_->state == steerState::STOP){
                    usedMotors[0].resetVelPid();
                    usedMotors[1].resetPosPid();
                    usedMotors[1].resetVelPid();
                    boardVal_->state = steerState::STOP;
                }   
            }
            // * 每次循环都固定发送一次can控制帧（但由于控制信息频率的限制，并不是每次发送的都是最新的信息）
            // todo 舵轮控制与PID计算
            if (boardCmd_->state > steerState::STOP && boardVal_->state < steerState::RESETOVER)
            {                    
                // 正常运行时进入这里，根据前面计算的偏差值，通过PID计算得到电流值并装填入发送缓存区
                if (steerCnt % motorTick == 0)
                {
                    // * dr1 计算电流，更新指令
                    dr1_tar_i = usedMotors[0].pidTick(PID_MODE::PID_MODE_VELOCITY, dr1_delta_v); // 这里的PID输出为 -10000 ~ 10000 量级的
                    // 转速控制时，目标电流通过计算值更新
                    curCmd[0] = (int)dr1_tar_i >> 8;
                    curCmd[1] = (int)dr1_tar_i & 0xFF;
                    // * dr2 计算轮速，电流，更新指令
                    // 复位过程中，轮速固定，其他过程中通过PID计算得到
                    if (boardVal_->state != steerState::RESETTING)
                        dr2_tar_i = usedMotors[1].pidTick(PID_MODE::PID_MODE_POSITION, dr2_delta_p);
                    else
                        dr2_tar_i = usedMotors[1].pidTick(PID_MODE::PID_MODE_VELOCITY, usedMotors[1].vel_tar - usedMotors[1].vel_current);

                    curCmd[2] = (int)dr2_tar_i >> 8;
                    curCmd[3] = (int)dr2_tar_i & 0xFF;
                    // 将后四个字节清零，一次CAN帧可以控制四个电机，这里只使用了两个
                    memset(curCmd + 4, 0, 4);   
                }
            }
            else
            {
                // 进入这里说明主机发送了STOP指令或是复位完成
                // 停机值
                usedMotors[0].vel_tar = 0.f;
                usedMotors[1].pos_tar = usedMotors[1].pos_current;
                usedMotors[1].vel_tar = 0.f;
                // 失能所有电机舵轮电机，也即curCmd的前四个字节
                memset(curCmd, 0, 4);
            }

            // ! 在这里插入第三个机构电机的控制代码
            // * 首先从传感器获取测量值
            for (int i = 0; i < 3 - boradType_; i++){
                // 主控制板（枚举值为1）有两个传感器，3-1=2
                // 辅助控制板（枚举值为2）有一个传感器，3-2=1
                // 只解析有效的传感器数据
                adcRealVal[i] = float(adcVal[i]) * adcCoeff[i] + adcOffset[i];
            }
            // 生成机构电机PID当前值与目标值
            if(boradType_ == BORAD_TYPE::MAIN_BOARD){
                usedMotors[2].pos_current = (adcRealVal[0] + adcRealVal[1]) / 2;
                usedMotors[2].pos_tar = boardCmd_->dr3_tar_tight;
            }else if(boradType_ == BORAD_TYPE::ASSIST_BOARD){
                usedMotors[2].pos_current = adcRealVal[0];
                usedMotors[2].pos_tar = boardCmd_->dr3_tar_angle;
            }
            
            // ! 机构电机控制结束

            // todo 发送can控制帧、存储反馈值，并判断是否发送
            CAN_SendMsg(CAN_Moto_ALL_ID, curCmd);
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