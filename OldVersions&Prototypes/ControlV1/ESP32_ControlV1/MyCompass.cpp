#include "MyCompass.h"
#include <Arduino.h>

const float ALPHA = 0.25; // Smoothing factor (0 < ALPHA < 1)
float ema_sin = 0.0;
float ema_cos = 0.0;

float degreesToRadians(float degrees) {
  return degrees * PI / 180.0;
}

float circularMean(float angles[], int size) {
  float sum_sin = 0.0;
  float sum_cos = 0.0;

  for (int i = 0; i < size; i++) {
    sum_sin += sin(degreesToRadians(angles[i]));
    sum_cos += cos(degreesToRadians(angles[i]));
  }

  float mean_angle = atan2(sum_sin / size, sum_cos / size);
  return fmod((mean_angle * 180.0 / PI + 360.0), 360.0);
}

float exponential_moving_average(float newAngle) { // EMA places higher weight on recent data than on older data - more reactive
  float newAngleRad = degreesToRadians(newAngle);
  ema_sin = (1 - ALPHA) * ema_sin + ALPHA * sin(newAngleRad);
  ema_cos = (1 - ALPHA) * ema_cos + ALPHA * cos(newAngleRad);
  float meanAngleRad = atan2(ema_sin, ema_cos);
  return fmod((meanAngleRad * 180.0 / PI + 360.0), 360.0);
}


float HeadingCalc(float xmag, float ymag, float zmag, float pitch, float roll){

  //Calibração ValCalib = A * (Val - b) - Hard Iron, Soft Iron, ortogonalidade 
  float xcal = 0, ycal = 0, zcal = 0;
  float b[3] = {-0.176509, -0.172115, 0.055995};
  float a1[3] = {1.090556, 0.001478, 0.002306};
  float a2[3] = {0.001478, 1.082508, 0.004032};
  float a3[3] = {0.002306, 0.004032, 1.109514};

  xcal = xmag - b[0];
  ycal = ymag - b[1];
  zcal = zmag - b[2]; 
  xcal = a1[0]*xcal + a1[1]*ycal + a1[2]*zcal;
  ycal = a2[0]*xcal + a2[1]*ycal + a2[2]*zcal;
  zcal = a3[0]*xcal + a3[1]*ycal + a3[2]*zcal;

  // Tilt compensation (project mag vectors into horizontal plane)
  float pitch_rad = pitch*M_PI/180; float roll_rad = roll*M_PI/180;
  float xh = xcal * cos(pitch_rad) + ycal * sin(roll_rad) * sin(pitch_rad) - zcal * cos(roll_rad) * sin(pitch_rad);
  float yh = ycal * cos(roll_rad) + zcal * sin(roll_rad);

  // Heading calculation through mag vectors projected in XY plane
  float heading = atan2(yh, xh);
  float declinationAngle = -0.0253072742; // Radeans, http://www.magnetic-declination.com/
  heading += declinationAngle;
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI - 180; 
  if (headingDegrees < 0 ){
    headingDegrees += 360;
  }

  float averageHeading = exponential_moving_average(headingDegrees);
  return averageHeading;
}