#ifndef __ADC_SENSOR_H_
#define __ADC_SENSOR_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include "main.h"

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus


#include "physparams.h"
#include "public_func.h"

#define ANGLE_SENSOR_MAX_V 4024.0f // ADC采样值最大值

// 將0~3.3V的ADC值转换为0~360度的角度值
const float adc_angle_coeff[4] = {
    360.0 / ANGLE_SENSOR_MAX_V, // 度每伏
    360.0 / ANGLE_SENSOR_MAX_V,
    360.0 / ANGLE_SENSOR_MAX_V,
    360.0 / ANGLE_SENSOR_MAX_V // 度每伏
};

const float adc_angle_offset[4] = {
    90.0f - 47.0f, // 偏移量
    90.0f - 162.0f, // 偏移量
    0.0f,
    0.0f // 偏移量
};

// ! 以下类暂时弃用

class ADC_SENSOR{
public:
    int8_t sensorNum_;
    uint32_t* adcVal_ori; // ADC原始采样值(大小为ADC通道数)
    float* adcVal; // ADC转换后的值(大小为ADC通道数)
    float* adcCoeff_; // ADC转换系数(大小为ADC通道数)
    float* adcOffset_; // ADC转换偏移量(大小为ADC通道数)
    ADC_HandleTypeDef *hadc_; // ADC句柄
    virtual float getSensorVal(){ return 0.f; } // 读取传感器值，默认实现
private:
};

class ANGLE_SENSOR : public ADC_SENSOR{
public:
    ANGLE_SENSOR(ADC_HandleTypeDef *hadc, float adcCoeff, float adcOffset);
    float getSensorVal() override;
};

class LENGTH_SENSOR : public ADC_SENSOR{
public:
    LENGTH_SENSOR(ADC_HandleTypeDef *hadc , float *adcCoeff, float *adcOffset);
    float getSensorVal() override;
};

// extern ADC_SENSOR* adcSensor;


#endif /* __cplusplus */





#endif /* __ADC_SENSOR_H_ */
