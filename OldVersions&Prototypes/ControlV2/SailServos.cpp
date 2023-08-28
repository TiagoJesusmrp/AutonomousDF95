#include "SailServos.h"
#include "Arduino.h"
#include "driver/ledc.h"

const int rudderPin = 32;
const int winchPin = 33;
const int rudderChan = 0;
const int winchChan = 1;
const int frequency = 50;


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
  int PulseWidthVal = map(WinchAngle, 0, 90, winch_pwm_min, winch_pwm_max); // Converts the Winch Angle from between 0-90 to PulseWidth
  return PulseWidthVal;
}

int ConvertRudderToPWM(float RudderAngle){
  // Takes Rudder Angle in Degrees and Converts to corresponding PulseWidth value for servo
  int PulseWidthVal;
  // Calculate the PWM value using linear interpolation
  PulseWidthVal = rudder_pwm_min + (RudderAngle + 45) * (rudder_pwm_max - rudder_pwm_min) / 90;
  return PulseWidthVal;
}
