/*
TIAGO JESUS
AUTONOMOUS DF95 MODEL SAILBOAT

Arduino NANO Code

Function: This code runs on the arduino NANO connected to the Radio Receiver Pins to measure the Pulse Width Values using interrupt routines and send them via UART to the ESP32.
          The Arduino NANO is also responsible for taking WindVane Sensor readings through SPI and sending them to ESP32.
*/

#define RCPin1 3  // Channel 3 on RX - Winch
volatile long StartTime1 = 0;
volatile long CurrentTime1 = 0;  // Volatile guarda na RAM em vez da storage normal para poder ser alterada por qq coisa a qq momento (interrupts) e o arduino deixar a variavel em paz sem a otimizar pois o seu valor pode ser alterado fora do loop
volatile long Pulses1 = 0;
int PulseWidth1 = 0;

#define RCPin2 2 // Channel 1 on RX - Rudder
volatile long StartTime2 = 0;
volatile long CurrentTime2 = 0;  
volatile long Pulses2 = 0;
int PulseWidth2 = 0;

#define RCPin3 4 // Channel 4 on RX - SD Reader Mission Commands (left stick sideways)
volatile long StartTime3 = 0;
volatile long CurrentTime3 = 0;  
volatile long Pulses3 = 49;
int PulseWidth3 = 50;

// Declarations for sending data through Serial
long LastTimeLog = 0;

#include <AS5048A.h>
AS5048A angleSensor(10);
uint16_t wvOffSet = 132;


void setup() {
  Serial.begin(9600);
  pinMode(RCPin1, INPUT_PULLUP);
  pinMode(RCPin2, INPUT_PULLUP);
  pinMode(RCPin3, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  angleSensor.begin();
  pinMode(10, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(RCPin1), PulseTimer1, CHANGE);  // Reads both HIGH and LOW pulse width and stores in Pulses variables
  attachInterrupt(digitalPinToInterrupt(RCPin2), PulseTimer2, CHANGE);  
}

void loop() {
  
  Pulses3 = getPulseWidth(RCPin3);

  if (Pulses1 < 2500 && Pulses1 > 900){ // Max HIGH Pulsewidth value is ~2ms (ignores LOW pulse width values)
    PulseWidth1 = Pulses1;
  }  
  if (Pulses2 < 2500 && Pulses2 > 900){ 
    PulseWidth2 = Pulses2;
  } 
  if (Pulses3 < 2500 && Pulses3 > 900){ 
    PulseWidth3 = Pulses3;
  }   

  // Send values through Serial
  int ssFreq = 10;
  if(millis() - LastTimeLog > 1000 / ssFreq){

    //Send Winch data
    Serial.print(PulseWidth1); 
    Serial.print(",");

    //Send Rudder data
    Serial.print(PulseWidth2);
    Serial.print(",");
    
    //Send mission Commands to ESP 
    if (PulseWidth3 > 1900){ // Turn stick to the max left
      Serial.print("s"); // Send Mission Start Command to ESP32
    }
    else if (PulseWidth3 < 1400){ // Turn stick to the max right
      Serial.print("e"); // Send Mission End Command to ESP32
    }
    else{
      Serial.print("i"); // Send Ignore Command to ESP32
    }

    // Taking windVane reading
    float val = 360 - angleSensor.getRotationInDegrees();
    if (val >= wvOffSet){
      val -= wvOffSet;
    }
    else {
      val += 360 - wvOffSet;
    }
    Serial.print(",");Serial.print(val);Serial.print("\n");


    LastTimeLog = millis();
  }  
  
}

// ISR's definitions (Interrupt Service Routine) - Functions to call when an interrupt is triggered
void PulseTimer1(){
  CurrentTime1 = micros();
  if (CurrentTime1 > StartTime1){
    Pulses1 = CurrentTime1 - StartTime1;
    StartTime1 = CurrentTime1;
  }
}

void PulseTimer2(){
  CurrentTime2 = micros();
  if (CurrentTime2 > StartTime2){
    Pulses2 = CurrentTime2 - StartTime2;
    StartTime2 = CurrentTime2;
  }
}

void PulseTimer3(){
  CurrentTime3 = micros();
  if (CurrentTime3 > StartTime3){
    Pulses3 = CurrentTime3 - StartTime3;
    StartTime3 = CurrentTime3;
  }
}

// PulseIn is used to read ModeSwitch Command. Not enough interrupt pins.
int getPulseWidth(int pin) {
  unsigned long pulseWidth = pulseIn(pin, HIGH, 20000);  // Timeout set to 20,000 microseconds (20ms)
  return pulseWidth;
}

