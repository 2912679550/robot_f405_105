/*
 * phys_params.h
 *
 *  Created on: Aug 7, 2014
 *Author: loywong
 */

#ifndef PHYSPARAMS_H_
#define PHYSPARAMS_H_

#define VERSION "V_1"
// ! 宏操作符
#define RAD2DEG(x) ((x) * (180.0f / 3.1415926f))
#define ABSF(x) ((x) < 0.0f ? -(x) : (x))
#define ABSI(x) ((x) < 0 ? -(x) : (x))
#define MAX(x, y) ((x) > (y) ? (x) : (y))
#define MIN(x, y) ((x) < (y) ? (x) : (y))

//================  system parameters
// ! 固定参数，一般不修改
const float PI = 3.14159265358979323846f;
// rad/s转换到rpm
const float toRPM = 9.55f;
// 系统控制周期
const float Ts = 0.001f;
// 电机控制周期
const float motorTs = 0.005f;
const int motorTick = 5; //(motorTs / Ts);
// CAN角度转换
const float canAngle = 8192.f / 360.f; //   0  ~ 8191 ->  0  ~ 360    将角度转换为圈
// CAN电流量程
const float C620Current = 16384.f;       // M3508 C620 -16384~16384 -> -20 ~ 20A
const float C610Current = 10000.f;       // M2006 C610 -10000~10000 -> -10 ~ 10A
const float C620ICoeff = 20.f / 16384.f; // M3508 C620 -16384~16384 -> -20 ~ 20A
const float C610ICoeff = 10.f / 10000.f; // M2006 C610 -10000~10000 -> -10 ~ 10A

// ! 车轮本身参数,根据实际情况修改参数
// 车轮半径
const float wheelR[2] = {0.05f , 0.05f}; // 0.05m
// 驱动电机减速比
const float wheelRatio[2] = {}; // 从电机转子rpm到车轮rpm       19.0f * 2.7551
const float steerRatio[2] = {}; // 从电机转子rpm到舵向rpm       19.0f * 2.7551

// 位置控制中最大线速度
const float maxVel = 0.5f; // 0.3m/s
// 位置控制中最大角速度
const float maxOmg = 1.f; // 1rad/s
// 舵轮控制中最大电流
const float maxCurrent = 9.5f; // M3508 C620 20A * 95%  = 19A // M2006 9.5

//================ end of system

// ! 舵轮控制PID参数
// * 舵轮转向位置环
const float thPosPidP[2] = 
    {10.f, 10.f}; // 从前到后为主驱动轮、辅助驱动轮的参数，暂时设置为相同
const float thPosPidI[2] = 
    {10.f, 10.f}; // 从前到后为主驱动轮、辅助驱动轮的参数，暂时设置为相同
const float thPosPidD[2] = 
    {0.f, 0.f}; // 从前到后为主驱动轮、辅助驱动轮的参数，暂时设置为相同
const float thPosPidIband[2] = 
    {PI / 10.f, PI / 10.f}; // 从前到后为主驱动轮、辅助驱动轮的参数，暂时设置为相同
// 舵轮控制中最大角速度
const float maxSteerOmg = PI; // 理论上最大11rad/s

// * 舵轮转向速度环
const float thVelPidP[2] = 
    {2000.f, 2000.f}; // 从前到后为主驱动轮、辅助驱动轮的参数，暂时设置为相同
const float thVelPidI[2] = 
    {15000.f, 15000.f}; // 从前到后为主驱动轮、辅助驱动轮的参数，暂时设置为相同
const float thVelPidD[2] = 
    {5000.0f, 5000.0f}; // 从前到后为主驱动轮、辅助驱动轮的参数，暂时设置为相同
const float thVelPidIband[2] = 
    {1.f, 1.f}; // 从前到后为主驱动轮、辅助驱动轮的参数，暂时设置为相同

// * 舵轮转速速度环
const float vVelPidP[2] = 
    {2.5f, 2.5f}; // 从前到后为主驱动轮、辅助驱动轮的参数，暂时设置为相同
const float vVelPidI[2] = 
    {25.0f, 25.0f}; // 从前到后为主驱动轮、辅助驱动轮的参数，暂时设置为相同
const float vVelPidD[2] = 
    {3.0f, 3.0f}; // 从前到后为主驱动轮、辅助驱动轮的参数，暂时设置为相同
const float vVelPidIband[2] = 
    {2000.0f, 2000.0f}; // 从前到后为主驱动轮、辅助驱动轮的参数，暂时设置为相同

// ! 机构执行电机参数
// * 机构电机位置环
const float mechPosPidP[2] = 
    {0.f, 10.f}; // 为主控板时，控制的是顶端丝杠，为辅助板时，控制的是一臂与二臂夹角
const float mechPosPidI[2] = 
    {0.f, 0.f}; // 为主控板时，控制的是顶端丝杠，为辅助板时，控制的是一臂与二臂夹角
// const float mechPosPidD[2] = 
//     {0.f, 0.f}; // 为主控板时，控制的是顶端丝杠，为辅助板时，控制的是一臂与二臂夹角
const float mechPosPidIband[2] = 
    {0.f, 0.f}; 
// * 机构电机速度环
const float mechVelPidP[2] = 
    {0.f, 0.f}; 
const float mechVelPidI[2] = 
    {0.f, 0.f};
const float mechVelPidD[2] = 
    {0.f, 0.f};
const float mechVelPidIband[2] = 
    {0.f, 0.f};

// ! ADC采样传感器到实际数据的转换系数
const float adcCoeff[2] = {0.001f, 0.001f}; // 0.001V/ADC
const float adcOffset[2] = {0.f, 0.f}; // 0.001V/ADC

// ! 用于标定舵轮角度值
// 注意在motor类中解析时是将这个值加在了电机的原始值上，需要注意正负符号
const double caliAngel_sensor[6] = {
    0.f, 0.f, 0.f, 0.f, 0.f, 0.f}; // 由激光传感器测得的标定原点
const double caliAngle_mech[6] = {
    0.5 * PI, 0.5 * PI, 0.5 * PI, 0.5 * PI, 0.5 * PI, 0.5 * PI}; // 由机械式方法（电机堵转）测得的标定原点

#endif /* PHYS_PARAMS_H_ */