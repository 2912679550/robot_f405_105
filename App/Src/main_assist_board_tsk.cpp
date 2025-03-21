#include "main_assist_board_tsk.h"
#include "ethernet_tsk.h"
#include "math.h"

namespace TskSteerBoard
{
    const int tskStkSize = 512;

    // SteerCmd *steerCmd_ = nullptr;       // 用于控制舵轮的命令
    // SteerRunValue *steerVal_ = nullptr;  // 用于反馈舵轮的状态

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
    float current_pos_[3] = {0.f}; // 舵轮和机构三个电机的当前位置信息
    float current_vel_[3] = {0.f}; // 舵轮和机构三个电机的当前速度信息
    float current_cur_[3] = {0.f}; // 舵轮和机构三个电机的当前电流信息

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
        Pid wheelSpeedPID(dr1VelPID_);
        // dr2 pos loop
        ffPid steerPosPID(dr2PosPID_);
        // dr2 vel loop
        Pid steerSpeedPID(dr2VelPID_);
        // dr3 pos loop
        if(dr3PosPID_ != nullptr){
            ffPid mechPosPID(dr3PosPID_);
        }
        // dr3 vel loop
        Pid mechSpeedPID(dr3VelPID_);

        // * 舵轮控制需要的量
        float dr1_delta_v = 0.f, dr2_delta_p = 0.f;
        float dr1_tar_i = 0.f, dr2_tar_i = 0.f;
        uint8_t zero = 1;
        uint8_t curCmd[8] = {0};
        // 上电时电流均给0
        CAN_SendMsg(CAN_Moto_ALL_ID, curCmd);

        // * 控制机构电机需要的量（暂无）

        while(true){
            rtn = xSemaphoreTake(mainAssistTickSem, 2);
            configASSERT(rtn);
            steerCnt++;

            // * 接收电机状态数据
            if (pdPASS == xQueueReceive(can1RxQueueHandle, motor_, 0))
            {
                if (motor_->id == 0)
                {
                    current_cur_[0] = motor_->given_current * C610ICoeff;
                    // 将电机rpm转换为轮子的线速度 , uint = m/s
                    current_vel_[0] = ((float)motor_->speed_rpm) * wheelR_ / wheelRatio_ / toRPM;
                }
                else if (motor_->id == 1)
                {
                    current_cur_[1] = motor_->given_current * C610ICoeff;
                    // 将电机rpm转换为舵轮的角速度,单位为rad/s
                    current_vel_[1] =  ((float)motor_->speed_rpm) / steerRatio_ / toRPM;
                    // 将电机的绝对角度转换为舵轮的角度,单位为rad
                    // 这里的 total_angle 为电调返回的直接值，大疆使用了0~8191来表示0~360度，所以数值最后还需要乘一个2pi/8192
                    current_pos_[1] = ((float)motor_->total_angle) / 8192.0f / steerRatio_  * 2.f * PI;
                }
                else if (motor_->id == 2)
                {
                    // 机构电机的电流和速度信息
                    current_cur_[2] = motor_->given_current * C610ICoeff;
                    current_vel_[2] = motor_->speed_rpm;
                    current_pos_[2] = 0.0;
                }
            }
            // 控制信息接收，boardCmd_中包含了舵轮的目标速度和位置以及主控制板的夹紧状态和辅助控制板的夹角
            xQueueReceive(TskEth::mainAssistCmdQueue, boardCmd_, 0);
            // todo 舵轮控制与PID计算
            if (boardCmd_->state > steerState::STOP && boardVal_->state < steerState::RESETOVER)
            {                    
                // 正常运行时进入这里
                if (steerCnt % motorTick == 0)
                {
                    // dr1 计算电流，更新指令
                    dr1_tar_i = wheelSpeedPID.Tick(dr1_delta_v);    // 这里的PID输出为 -10000 ~ 10000 量级的
                    // 转速控制时，目标电流通过计算值更新
                    // boardCmd_->dr1_tar_cur = dr1_tar_i * C610ICoeff;
                    curCmd[0] = (int)dr1_tar_i >> 8;
                    curCmd[1] = (int)dr1_tar_i & 0xFF;
                    // dr2 计算轮速，电流，更新指令
                    // 复位过程中，轮速固定，其他过程中通过PID计算得到
                    if (boardVal_->state != steerState::RESETTING)
                        boardVal_->dr2_tar_vel = steerPosPID.Tick(dr2_delta_p, 0.f);
                    dr2_tar_i = steerSpeedPID.Tick(boardVal_->dr2_tar_vel - current_vel_[1]);
                    // boardVal_->dr2_tar_cur = dr2_tar_i * C610ICoeff;
                    curCmd[2] = (int)dr2_tar_i >> 8;
                    curCmd[3] = (int)dr2_tar_i & 0xFF;
                    // 将后四个字节清零，一次CAN帧可以控制四个电机，这里只使用了两个
                    memset(curCmd + 4, 0, 4);   
                }
            }
            else
            {
                // 停机值
                boardCmd_->dr1_tar_vel = 0.f;
                boardCmd_->dr2_tar_pos = boardVal_->dr2_real_pos;
                boardVal_->dr2_tar_vel = 0.f;
                // 失能所有电机
                memset(curCmd, 0, 8);
            }

            // todo 机构电机控制与PID计算
            CAN_SendMsg(CAN_Moto_ALL_ID, curCmd);
        }

    }



};