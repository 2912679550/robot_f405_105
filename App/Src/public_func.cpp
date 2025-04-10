#include "public_func.h"

// linear velocity (m/s) to motor rotate speed (rpm) 
// // inline float dr1_vel2rpm(const float vel)
// // {
// //     return vel * ratio * toRPM / wheelR;
// // }
//  motor rotate speed (rpm) to linear velocity (m/s)
// //inline float dr1_rpm2vel(int16_t rpm)
// //{
// //    return ((float)rpm) * wheelR / ratio / toRPM;
// //}
// motor angle to distance (m)
// //inline float dr1_ang2dis(int32_t total_angle)
// //{
// //    return ((float)total_angle) / 8192.0f / ratio * 2.f * PI * wheelR;
// //}

// motor angle to wheel angle (rad)
// // inline float dr2_angConvert(int32_t total_angle)
// // {
// //     return ((float)total_angle) / 8192.0f / ratio2 * 2.f * PI;
// // }
// motor rotate speed (rpm) to wheel rotate speed (rad/s)
// inline float dr2_rpmConvert(int16_t rpm)
// {
//     return ((float)rpm) / ratio2 / toRPM;
// }
float saturate(float v, float max, float min)
{
    return v > max ? max : v < min ? min
                                   : v;
}
float cacul_ppi_angle(const float tar, const float cur)
{
    return (tar - cur < -PI) ? (tar - cur + 2 * PI) : (tar - cur > PI) ? (tar - cur - 2 * PI)
                                                                       : (tar - cur);
}

float angle_stand_deg(float angle){
    while(angle < 0.0f){
        angle += 360.0f;
    }
    while(angle > 360.0f){
        angle -= 360.0f;
    }
    return angle;
}

float angle_stand_rad(float angle){
    while(angle < 0.0f){
        angle += 2 * PI;
    }
    while(angle > 2 * PI){
        angle -= 2 * PI;
    }
    return angle;
}




