#ifndef __PID_H__
#define __PID_H__

typedef struct{
    float p, i, d, n, ts;           // PID参数 分别为：比例、积分、微分、采样周期
    float outIMin, outIMax, Iband;  // 积分限幅、积分死区
    float outMin, outMax;           // 输出限幅
} PID_PARAM;

typedef enum
{
    PID_MODE_IDLE,
    PID_MODE_POSITION,
    PID_MODE_VELOCITY,
    PID_MODE_TORQUE,
}PID_MODE;

#ifdef __cplusplus

class Pid
{
private:
float accI, accD, accIMax, accIMin, accDMax, accDMin, outMax, outMin, Iband;
public:
    float p, i, d, n, ts;
    Pid(float p, float i, float d, float n, float ts,  float outIMin, float outIMax, float Iband, float outMin, float outMax);
    Pid(PID_PARAM *param);
    float Tick(float diff);
    void Reset();
    void SetParam(float p, float i);
};
#endif
#endif
