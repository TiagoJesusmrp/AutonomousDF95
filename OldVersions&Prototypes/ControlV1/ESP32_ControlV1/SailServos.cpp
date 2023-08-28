#include "SailServos.h"
#include "Arduino.h"
#include "driver/ledc.h"

const int rudderPin = 32;
const int winchPin = 33;
const int rudderChan = 0;
const int winchChan = 1;
const int frequency = 50;
int pulseWidth1 = 0; // In microseconds for servo 1
const int pulseWidth2 = 1800; // In microseconds for servo 2

void PWMChanSetup(){
  // Configure the PWM pins for servos
  ledcSetup(rudderChan, frequency, 8); // Channel 0, 8-bit resolution
  ledcSetup(winchChan, frequency, 8); // Channel 1, 8-bit resolution
  ledcAttachPin(rudderPin, 0); // Attach Channel 0 to rudder Pin
  ledcAttachPin(winchPin, 1); // Attach Channel 1 to winch Pin
}

void SetRudderPosition(int PulseWidthVal){
  int dutyCycle = round(PulseWidthVal * 255 / 20000); // Convert pulse width to duty cycle value - at 50hz 20000us corresponds to 100% duty cycle
  ledcWrite(rudderChan, dutyCycle);
}

void SetWinchPosition(int PulseWidthVal){
  int dutyCycle = round(PulseWidthVal * 255 / 20000); // Convert pulse width to duty cycle value - at 50hz 20000us corresponds to 100% duty cycle
  ledcWrite(winchChan, dutyCycle);
}

int ConvertWinchToPWM(float WinchAngle){
  // Takes Winch Angle in Degrees and Converts to corresponding PulseWidth value for servo
  int PulseWidthVal = map(WinchAngle, 0, 90, 1308, 2056); // Converts the Winch Angle from between 0-90 to PulseWidth
  return PulseWidthVal;
}

int ConvertRudderToPWM(float RudderAngle){
  // Takes Rudder Angle in Degrees and Converts to corresponding PulseWidth value for servo
  int PulseWidthVal;
  const int pwm_min = 2040; // PWM value at -45 degrees
  const int pwm_max = 1216; // PWM value at 45 degrees
  // Calculate the PWM value using linear interpolation
  PulseWidthVal = pwm_min + (RudderAngle + 45) * (pwm_max - pwm_min) / 90;
  return PulseWidthVal;
}
