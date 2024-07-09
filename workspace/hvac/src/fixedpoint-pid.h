#ifndef __FIXED_PID_H__
#define __FIXED_PID_H__
#include <stdint.h>
typedef int32_t Fixed32;
typedef int64_t Fixed64;
typedef struct {
	Fixed32 Dt;
	Fixed32 Max;
	Fixed32 Min;
	Fixed32 Kp;
	Fixed32 Kd;
	Fixed32 Ki;
	Fixed32 PrevError;
	Fixed32 Integral;
} FixedPid;
/******************************/
Fixed32 Fixed32_FromInt(int32_t n);
int32_t Fixed32_Frac(Fixed32 a);
Fixed32 Fixed32_FromFloat(float f);
Fixed32 Fixed32_ToFloat(float T);
Fixed32 Fixed32_Mul(Fixed32 a, Fixed32 b);
Fixed32 Fixed32_Div(Fixed32 a, Fixed32 b);
Fixed32 FixedPid_Calculate(FixedPid* self, Fixed32 setpoint, Fixed32 pv);
void FixedPid_Init(FixedPid* self); 
/*****************************/
#endif 

