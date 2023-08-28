#ifndef SAILSERVOS_H
#define SAILSERVOS_H
#include "Arduino.h"
#include "driver/ledc.h"

extern int rudder_pwm_min;
extern int rudder_pwm_max;
extern int winch_pwm_min;
extern int winch_pwm_max;

void PWMChanSetup(void);
void SetRudderPosition(int PulseWidthVal);
void SetWinchPosition(int PulseWidthVal);
int ConvertWinchToPWM(float WinchAngle);
int ConvertRudderToPWM(float RudderAngle);

#endif