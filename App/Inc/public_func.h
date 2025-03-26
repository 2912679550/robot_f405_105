#ifndef __PUBLIC_FUNC_H_
#define __PUBLIC_FUNC_H_

// * 引用C的头文件
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

// linear velocity (m/s) to motor rotate speed (rpm)
// // inline float dr1_vel2rpm(const float vel);
//  motor rotate speed (rpm) to linear velocity (m/s)
// //inline float dr1_rpm2vel(int16_t rpm);
// motor angle to distance (m)
// //inline float dr1_ang2dis(int32_t total_angle);
// motor angle to wheel angle (rad)
// // inline float dr2_angConvert(int32_t total_angle);
// motor rotate speed (rpm) to wheel rotate speed (rad/s)
// // inline float dr2_rpmConvert(int16_t rpm);
float saturate(float v, float max, float min);
float cacul_ppi_angle(const float tar, const float cur);

#endif /* __cplusplus */



#endif /* __PUBLIC_FUNC_H_ */


