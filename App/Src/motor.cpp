#include "motor.h"
#include "physparams.h"

MOTOR usedMotors[3]; // 0:dr1 1:dr2 2:dr3

void MOTOR::initMotorPid(PID_PARAM *speedPidParam, FFPID_PARAM *posPidParam){
    if(posPidParam != nullptr)  pos_pid = new ffPid(posPidParam);
    if(speedPidParam != nullptr) speed_pid = new Pid(speedPidParam);
}

inline void MOTOR::resetPosPid(){
    if(pos_pid != nullptr) pos_pid->Reset();
}


inline void MOTOR::resetVelPid(){
    if(speed_pid != nullptr) speed_pid->Reset();
}

float MOTOR::pidTick(PID_MODE mode, float delta){
    float out2can = 0.f;
    ctrl_mode = mode;
    float deltaV = delta; 
    if(mode == PID_MODE::PID_MODE_POSITION){
        // 首先位置环输出得到期望速度
        vel_tar = pos_pid->Tick(delta, 0.f);
        deltaV = vel_tar - vel_current;
    }else{
        vel_tar = vel_current + deltaV; // 这里的deltaV是电机转速环的输入值
    }

    if(mode >= PID_MODE::PID_MODE_POSITION){
        out2can = speed_pid->Tick(deltaV); // 这里的out是对应电机电调的电流数据格式
        cur_tar = out2can * can2cur; // 这里的out是对应电机电调的电流数据格式
    }
    return out2can;
}

void MOTOR::unpackCanData(moto_measure_t *motorData){
    if(motorData == nullptr) return;
    pos_current = motorData->total_angle * can2pos; // 这里的total_angle是电调返回的直接值，大疆使用了0~8191来表示0~360度，所以数值最后还需要乘一个2pi/8192
    vel_current = motorData->speed_rpm * can2vel; // 将电机rpm转换为轮子的线速度 , uint = m/s
    cur_current = motorData->given_current * can2cur;
}











