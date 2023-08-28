#include "MyCompass.h"
#include <TinyGPSPlus.h>
#include <Arduino.h>
extern TinyGPSPlus gps;

const float ALPHA = 0.25; // Smoothing factor (0 < ALPHA < 1)
float ema_sin = 0.0;
float ema_cos = 0.0;

float degreesToRadians(float degrees) {
  return degrees * PI / 180.0;
}

float exponential_moving_average(float newAngle) { // EMA places higher weight on recent data than on older data - more reactive
  float newAngleRad = degreesToRadians(newAngle); // Averages are calculated using radians and sin/cos to account for wrap around 360-0
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


#include <math.h>

// Define the Earth's radius in kilometers
#define EARTH_RADIUS 6371

// Define conversion factors between degrees and radians
#define DEG_TO_RAD (M_PI / 180.0)
#define RAD_TO_DEG (180.0 / M_PI)

// Function to convert Geographic coordinates to Cartesian coordinates
void geoToCart(double lat, double lon, double& x, double& y, double& z) {
  double latRad = lat * DEG_TO_RAD;
  double lonRad = lon * DEG_TO_RAD;

  x = EARTH_RADIUS * cos(latRad) * cos(lonRad);
  y = EARTH_RADIUS * cos(latRad) * sin(lonRad);
  z = EARTH_RADIUS * sin(latRad);
}

// Function to convert Cartesian coordinates to Geographic coordinates
void cartToGeo(double x, double y, double z, double& lat, double& lon) {
  lat = atan2(z, sqrt(x * x + y * y)) * RAD_TO_DEG;
  lon = atan2(y, x) * RAD_TO_DEG;
}

// Function to perform the orthogonal projection of C into AB line and return the distance between C and projection
float distanceToProjectPoint(double latA, double lonA, double latB, double lonB, double latC, double lonC) {
  double ax, ay, az, bx, by, bz, cx, cy, cz;
  
  geoToCart(latA, lonA, ax, ay, az);
  geoToCart(latB, lonB, bx, by, bz);
  geoToCart(latC, lonC, cx, cy, cz);
  
  // Vector from A to B
  double abx = bx - ax;
  double aby = by - ay;
  double abz = bz - az;
  
  // Vector from A to C
  double acx = cx - ax;
  double acy = cy - ay;
  double acz = cz - az;
  
  // Scalar projection of AC onto AB
  double scalarProj = (acx * abx + acy * aby + acz * abz) / 
                      sqrt(abx * abx + aby * aby + abz * abz);
  
  // Cartesian coordinates of the orthogonal projection
  double px = ax + scalarProj * abx;
  double py = ay + scalarProj * aby;
  double pz = az + scalarProj * abz;
  
  // Convert back to Geographic coordinates
  double latP, lonP;
  cartToGeo(px, py, pz, latP, lonP);
  
  float distanceToProjection = gps.distanceBetween(latC, lonC, latP, lonP);
  return distanceToProjection;
}