#ifndef RUDDERCONTROLLER_H
#define RUDDERCONTROLLER_H
#include "Arduino.h"
#include "Fuzzy.h"

void RudderControllerSetup();
float GetRudderControlAction(float inputError);
float calculateHeadingError(float currentHeading, float desiredHeadingToTarget);

#endif