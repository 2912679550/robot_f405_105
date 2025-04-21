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
#define PI  3.14159265358979323846f
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
/* 来自余工：
    主动轮：
        轮电机：
            带轮36：14；齿轮73.6：9.6；总传动比19.71：1。
            直径100mm
            电机1Nm，效率0.8，驱动力1*19.71*0.8/0.05=315.36N。
        舵电机：
            主动轮舵向传动比：125:15
    辅助轮：
        轮电机：
            带轮40：14；
            轮子直径60mm，
            驱动力 1*40/14/0.03*0.9=85N。
        舵电机：
            辅助轮舵向传动比：81:15
*/
// 车轮半径，数组ID依次为主驱动轮、辅助驱动轮
#define M2006_RATIO  36.0f // M2006电机减速比
const float wheelR[2] = { 100.0 / 1000.0 * 0.5 , 60.0 / 1000.0 * 0.5}; // 
// 驱动电机减速比
const float wheelRatio[2] = {M2006_RATIO * 19.71 , M2006_RATIO * 40.0 / 14.0}; // 从电机转子rpm到车轮rpm       19.0f * 2.7551
const float steerRatio[2] = {M2006_RATIO * 125.0/15.0 , M2006_RATIO * 81.0 / 15.0}; // 从电机转子rpm到舵向rpm       19.0f * 2.7551

// 机器人运动的最大线速度
const float maxVel = 0.2f; // 0.3m/s
// 位置控制中最大角速度
const float maxOmg = 1.f; // 1rad/s
// 舵轮控制中最大电流
const float maxCurrent = 9.5f; // M3508 C620 20A * 95%  = 19A // M2006 9.5

//================ end of system

// ! 舵轮控制PID参数
// * 舵电机位置环
const float thPosPidP[2] = 
    {5.f, 1.8f}; // 从前到后为主驱动轮、辅助驱动轮的参数，暂时设置为相同
const float thPosPidI[2] = 
    {1.f, 1.2f}; // 从前到后为主驱动轮、辅助驱动轮的参数，暂时设置为相同
const float thPosPidD[2] = 
    {0.f, 0.f}; // 从前到后为主驱动轮、辅助驱动轮的参数，暂时设置为相同
const float thPosPidIband[2] = 
    {PI / 10.f, PI / 10.f}; // 从前到后为主驱动轮、辅助驱动轮的参数，暂时设置为相同
// 舵轮控制中最大角速度
const float maxSteerOmg = PI; // 理论上最大11rad/s
const float maxScrewOmg = 20.0f; // 限制丝杠最大速度


// * 舵电机速度环
const float thVelPidP[2] = 
    {3000.f, 3000.f}; // 从前到后为主驱动轮、辅助驱动轮的参数，暂时设置为相同
const float thVelPidI[2] = 
    {1000.f, 1000.f}; // 从前到后为主驱动轮、辅助驱动轮的参数，暂时设置为相同
const float thVelPidD[2] = 
    {500.0f, 500.0f}; // 从前到后为主驱动轮、辅助驱动轮的参数，暂时设置为相同
const float thVelPidIband[2] = 
    {1.f, 1.f}; // 从前到后为主驱动轮、辅助驱动轮的参数，暂时设置为相同

// * 轮电机速度环
const float vVelPidP[2] = 
    {65000.f, 40000.0f}; // 从前到后为主驱动轮、辅助驱动轮的参数，暂时设置为相同
const float vVelPidI[2] = 
    {2000.0f, 2000.0f}; // 从前到后为主驱动轮、辅助驱动轮的参数，暂时设置为相同
const float vVelPidD[2] = 
    {1000.0f, 1200.0f}; // 从前到后为主驱动轮、辅助驱动轮的参数，暂时设置为相同
const float vVelPidIband[2] = 
    {2000.0f, 2000.0f}; // 从前到后为主驱动轮、辅助驱动轮的参数，暂时设置为相同

// ! 机构执行电机参数
// * 机构电机位置环
const float mechPosPidP[2] = 
    {0.f, 5.f}; // 为主控板时，控制的是顶端丝杠，为辅助板时，控制的是一臂与二臂夹角
const float mechPosPidI[2] = 
    {0.f, 2.f}; // 为主控板时，控制的是顶端丝杠，为辅助板时，控制的是一臂与二臂夹角
const float mechPosPidIband[2] = 
    {0.f, 10.f}; 
// * 机构电机速度环
const float mechVelPidP[2] = 
    {250.f, 150.f}; 
const float mechVelPidI[2] = 
    {200.f, 50.f};
const float mechVelPidD[2] = 
    {120.f, 5.f};
const float mechVelPidIband[2] = 
    {500.f, 500.f};

// ! 用于标定舵轮角度值
// 注意在motor类中解析时是将这个值加在了电机的原始值上，需要注意正负符号
const double caliAngel_sensor[6] = {
    0.f, 0.f, 0.f, 0.f, 0.f, 0.f}; // 由激光传感器测得的标定原点
const double caliAngle_mech[6] = {
    0.0 * PI, 0.0 * PI, 0.0 * PI, 0.0 * PI, 0.0 * PI, 0.0 * PI}; // 由机械式方法（电机堵转）测得的标定原点

// ! 机构电机的安装方向
const float wheelDir[6] = {
    1.f, -1.f, -1.f, 1.f, -1.f, 1.f}; // 轮电机旋转方向与数学定义的关系 usedMotors[0]
const float steerDir[6] = {
    1.f, 1.f, 1.f, 1.f, 1.f, 1.f}; // 舵向电机旋转方向与数学定义的关系 usedMotors[1]
const float mechDir[6] = {
    1.f, 1.f, 1.f, 1.f, 1.f, 1.f}; // 机构电机选装方向与数学定义的关系 usedMotors[2]

// ! 用于定义用到的一些运动范围
// 舵电机标定后的角度范围
const float steerDirRange[2] = 
    {0.02 * PI, PI}; // 机械标定的形式最小值 0 rad处会有碰撞，所以加一点保护，而最大值PI处是不会有机械限位的
// 两臂家教的输入角度范围：
// 夹360mm管子时，臂的角度124.6°
// 夹220mm管子时，臂的角度61°
const float mechAngleRange[2] = {
    180.0f - 125.0f,
    180.0f - 61.0f
}; // 两臂夹角传感器的0点值被定义为模型中直观感觉的补角，单位为度

const float mechSpringMin = 30.0f;  // 期望夹紧弹簧达到预期夹紧力时的最小长度
const float mechSpringMax = 50.0f;  // 期望夹紧弹簧达到预期夹紧力时的最大长度

#endif /* PHYS_PARAMS_H_ */