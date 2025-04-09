#include "main.h"
#include "adcSensor.h"

// 用于存储ADC采样值，结合main文件中对ADC通道的配置：
// PC0：ADC1_IN10  rank 1
// PC1：ADC1_IN11  rank 2
// PC2：ADC1_IN12  rank 3
ADC_SENSOR *adcSensor;
// HAL_ADC_Start_DMA(&hadc1 , (uint32_t *)adcVal, 2); // 启动ADC采样，读取电压值
ANGLE_SENSOR::ANGLE_SENSOR(ADC_HandleTypeDef *hadc , float adcCoeff, float adcOffset)
{
    // * 传感器的ADC采样值
    sensorNum_ = 1;
    hadc_ = hadc;
    adcVal_ori = (uint32_t *)pvPortMalloc(sensorNum_ * sizeof(uint32_t));
    adcVal = (float *)pvPortMalloc(sensorNum_ * sizeof(float));
    adcCoeff_ = (float *)pvPortMalloc(sensorNum_ * sizeof(float));
    adcOffset_ = (float *)pvPortMalloc(sensorNum_ * sizeof(float));
    adcCoeff_[0] = adcCoeff;
    adcOffset_[0] = adcOffset;
    if (adcVal_ori == nullptr || adcVal == nullptr || adcCoeff_ == nullptr || adcOffset_ == nullptr)
        return;
}


float ANGLE_SENSOR::getSensorVal()
{
    // * 读取ADC采样值
    uint16_t tempVel[3] = {0};
    HAL_ADC_Start_DMA(&hadc1 , (uint32_t *)tempVel, 3);
    for (int i = 0; i < sensorNum_; i++)
    {
        adcVal_ori[i] = tempVel[i]; // 读取ADC采样值
        adcVal[i] = float(adcVal_ori[i]) * adcCoeff_[i] - adcOffset_[i];
    }
    return adcVal[0]; // 返回第一个传感器的值
}

LENGTH_SENSOR::LENGTH_SENSOR(ADC_HandleTypeDef *hadc , float *adcCoeff, float *adcOffset)
{
    // * 传感器的ADC采样值
    sensorNum_ = 2;
    hadc_ = hadc;
    adcVal_ori = (uint32_t *)pvPortMalloc(sensorNum_ * sizeof(uint32_t));
    adcVal = (float *)pvPortMalloc(sensorNum_ * sizeof(float));
    adcCoeff_ = (float *)pvPortMalloc(sensorNum_ * sizeof(float));
    adcOffset_ = (float *)pvPortMalloc(sensorNum_ * sizeof(float));
    if (adcVal_ori == nullptr || adcVal == nullptr || adcCoeff_ == nullptr || adcOffset_ == nullptr)
        return;
    for (int i = 0; i < sensorNum_; i++)
    {
        adcCoeff_[i] = adcCoeff[i];
        adcOffset_[i] = adcOffset[i];
    }
}

float LENGTH_SENSOR::getSensorVal()
{
    // * 读取ADC采样值
    uint16_t tempVel[3] = {0};
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)tempVel, 3);
    for (int i = 0; i < sensorNum_; i++)
    {
        adcVal_ori[i] = tempVel[i]; // 读取ADC采样值
        adcVal[i] = float(adcVal_ori[i]) * adcCoeff_[i] - adcOffset_[i];
    }
    return (adcVal[0] + adcVal[1]) / 2; // 返回平均值
}
