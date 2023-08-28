#ifndef MYCOMPASS_H
#define MYCOMPASS_H
#include <Arduino.h>
#include <TinyGPSPlus.h>
extern TinyGPSPlus gps;


float HeadingCalc(float xmag, float ymag, float zmag, float pitch, float roll);
float distanceToProjectPoint(double latA, double lonA, double latB, double lonB, double latC, double lonC);


#endif