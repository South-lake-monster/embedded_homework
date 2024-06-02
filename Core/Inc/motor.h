#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f4xx_hal.h"
#include "math.h"
#include "tim.h"
#include "stm32f4xx_it.h"


//初始化变量及定义
#define PI 3.14159265358979323846f
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

float _electricalAngle(float shaft_angle, int pole_pairs);
float _normalizeAngle(float angle);
void setPwm(float Ua, float Ub, float Uc);
void setPhaseVoltage(float Uq,float Ud, float angle_el);
float velocityOpenloop(float target_velocity);

#endif // __MOTOR_H
