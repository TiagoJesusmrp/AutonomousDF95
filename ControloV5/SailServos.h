#ifndef SAILSERVOS_H
#define SAILSERVOS_H
#include "Arduino.h"
#include "driver/ledc.h"


void PWMChanSetup(void);
void SetRudderPosition(int PulseWidthVal);
void SetWinchPosition(int PulseWidthVal);
int ConvertWinchToPWM(float WinchAngle);
int ConvertRudderToPWM(float RudderAngle);

#endif