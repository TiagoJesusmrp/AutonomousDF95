#ifndef MYCOMPASS_H
#define MYCOMPASS_H
#include <Arduino.h>

float HeadingCalc(float xmag, float ymag, float zmag, float pitch, float roll);

#endif