#include "main.h"
#include "motor_tsk.h"

namespace TskMotorPID
{
    const int tskStkSize = 512;
    uint8_t tskPeriod = 2; // ms

    // * PID参数
    PID_PARAM* dr1VelPID_ = nullptr;      // 舵轮轮电机，跑速度环
    FFPID_PARAM* dr2PosPID_ = nullptr;    // 舵轮转向电机，跑位置环
    PID_PARAM* dr2VelPID_ = nullptr;
    FFPID_PARAM* dr3PosPID_ = nullptr;    // 用于控制机构电机的前置PID，用于处理传感器测量位置数据到电机期望速度的转换
    PID_PARAM* dr3VelPID_ = nullptr;      // 用于执行机构电机的速度环PID控制\

    // * 结构参数
    float wheelRatio_ = 0.f;    // 轮电机电机的减速比
    float steerRatio_ = 0.f;    // 舵电电机的减速比
    float wheelR_ = 0.f;        // 舵轮轮胎半径

    // * 信息容器
    moto_measure_t *motor_ = nullptr;
    int TCP_IP_ID = 0;
    BORAD_TYPE boradType_ = BORAD_TYPE::idle; // 控制板类型(这个任务中只会设置为主驱动轮或辅助驱动轮)
    uint32_t motorCnt = 0;

    void Init(){
        BaseType_t rtn;
        
        load_params_(); // 读取参数
        load_motor_params_(); // 装载电机类的机械参数
        
        motor_ = (moto_measure_t *)pvPortMalloc(sizeof(moto_measure_t));
        if (motor_ == nullptr)
            return;
        // 创建电机PID解算与控制任务
        rtn = xTaskCreate(motorTickTask, (const portCHAR *)"motorTickTask",
                        tskStkSize, NULL, osPriorityAboveNormal, NULL);
        configASSERT(rtn == pdPASS);
    }

    void motorTickTask(void *pvParameters)
    {
        BaseType_t rtn;
        can1RxQueueHandle = xQueueCreate(6, sizeof(moto_measure_t));
        // 初始化can并使能中断
        CAN_Start_Trans();
        uint8_t curCmd[8] = {0};
        // 上电时电流均给0
        // CAN_SendMsg(CAN_Moto_ALL_ID, curCmd);

        while(true){
            vTaskDelay(tskPeriod); // 
            // rtn = xSemaphoreTake(motorTickSem, 2);
            // configASSERT(rtn);
            motorCnt++;
            

            // * 接收电机数据
            if (pdPASS == xQueueReceive(can1RxQueueHandle, motor_, 0) && motor_ -> id < MOTOR_NUM)
                usedMotors[motor_->id].unpackCanData(motor_);   // 此时已经将can帧的消息解包到电机类中
            
            
            // 清空can帧缓存区
            memset(curCmd, 0, sizeof(curCmd));
            float delta_p = 0.f;
            float delta_v = 0.f;
            float tar_i = 0.f;

            // * 每次循环都固定发送一次can控制帧（但由于控制信息频率的限制，并不是每次发送的都是最新的信息）
            for(int i = 0; i < MOTOR_NUM; i++){
                // 遍历所有需要使用的电机，根据电机的控制模式解算出对应的电流值，装填到can帧中
                switch(usedMotors[i].ctrl_mode){
                    case PID_MODE::PID_MODE_IDLE:
                        usedMotors[i].pos_tar = usedMotors[i].pos_current;
                        usedMotors[i].vel_tar = usedMotors[i].vel_current;
                        usedMotors[i].cur_tar = 0.f;
                        tar_i = 0.f;
                        break;
                    case PID_MODE::PID_MODE_POSITION:
                        delta_p = usedMotors[i].pos_tar - usedMotors[i].pos_current; // 位置环的输入值
                        tar_i = usedMotors[i].pidTick(PID_MODE::PID_MODE_POSITION, delta_p); // 这里的PID输出为 -10000 ~ 10000 量级的
                        break;
                    case PID_MODE::PID_MODE_VELOCITY:
                        delta_v = usedMotors[i].vel_tar - usedMotors[i].vel_current; // 速度环的输入值
                        tar_i = usedMotors[i].pidTick(PID_MODE::PID_MODE_VELOCITY, delta_v); // 这里的PID输出为 -10000 ~ 10000 量级的
                        break;
                    case PID_MODE::PID_MODE_TORQUE:
                        break;
                    default:
                        usedMotors[i].pos_tar = usedMotors[i].pos_current;
                        usedMotors[i].vel_tar = usedMotors[i].vel_current;
                        usedMotors[i].cur_tar = 0.f;
                        usedMotors[i].work_log = WORKING_LOG::NORMAL; // 电机工作状态恢复正常
                        tar_i = 0.f;
                        break;
                    }
                    curCmd[i * 2] = (int)tar_i >> 8; // 高8位
                    curCmd[i * 2 + 1] = (int)tar_i & 0xFF; // 低8位
            }
            // if(motorCnt % motorTick != 0) continue; // 限定电机控制的周期
            CAN_SendMsg(CAN_Moto_ALL_ID, curCmd);
        }
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
            dr3PosPID_->outIMin = -maxScrewOmg / 5;
            dr3PosPID_->outIMax = maxScrewOmg / 5;
            dr3PosPID_->Iband = mechPosPidIband[index];
            dr3PosPID_->outMin = -maxScrewOmg;
            dr3PosPID_->outMax = maxScrewOmg;
            
            usedMotors[2].outer_pos = true; // 机构电机使用外部传感器的测量值作为位置值
        }
        else if(boradType_ == BORAD_TYPE::MAIN_BOARD){
            // 配置夹紧电机的堵转电流
            dr3PosPID_ = nullptr; // 主控板的机构电机不需要位置环
            usedMotors[2].block_cur = 7.0f;
        }
        
        // * 装载到电机类中
        usedMotors[0].initMotorPid(dr1VelPID_, nullptr); // dr1 速度环
        usedMotors[1].initMotorPid(dr2VelPID_, dr2PosPID_); // dr2 速度环和位置环
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
        usedMotors[2].can2vel = 1/ toRPM / M2006_RATIO; // dr3 机构电机，单位为rad/s
        usedMotors[2].can2pos = 0; // dr3 机构电机,单位为rad

        // * 配置旋转方向
        usedMotors[0].mech_dir = wheelDir[TCP_IP_ID];
        usedMotors[1].mech_dir = steerDir[TCP_IP_ID];
        usedMotors[2].mech_dir = mechDir[TCP_IP_ID]; // 机构电机的方向与电机的安装方向一致

    }

} // namespace TskMotorPID



