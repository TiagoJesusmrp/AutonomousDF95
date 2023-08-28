#ifndef FUZZYCONTROLLER_H
#define FUZZYCONTROLLER_H
#include "Arduino.h"
#include "Fuzzy.h"

void WinchControllerSetup();
float GetWinchControlAction(float relativeWind);
void RudderControllerSetup();
float GetRudderControlAction(float inputError);
float calculateHeadingError(float currentHeading, float desiredHeadingToTarget);
int mapToNegative(int val);

#endif