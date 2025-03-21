#ifndef __FEEDFORWARDPID_H__
#define __FEEDFORWARDPID_H__

typedef struct
{
    float p, i, wc, ts;           // PID参数 分别为：比例、积分、微分、采样周期
    float outIMin, outIMax, Iband;  // 积分限幅、积分死区
    float outMin, outMax;           // 输出限幅
} FFPID_PARAM;

#ifdef __cplusplus
class ffPid
{
public:
    float p, i, wc, ts, pre_tar, pre_tar_dot_filtered, tar_dot;
    float accI, accIMax, accIMin, outMax, outMin, Iband;

public:
    ffPid(float p, float i, float wc, float ts, float outIMin, float outIMax, float Iband, float outMin, float outMax);
    ffPid(FFPID_PARAM *param);
    float Tick(float diff, float tar);
    void Reset();
};
#endif
#endif
