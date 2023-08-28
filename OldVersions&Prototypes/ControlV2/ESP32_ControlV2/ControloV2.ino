// Declarations for the GPS sensor
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#define gpsSSRXPin 25
#define gpsSSTXPin 26
double gpsLastT = 0;
float currentLat = -1;
float currentLng = -1;
float Speed, Course, Time;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial SerialGPS(gpsSSRXPin, gpsSSTXPin);

// Declarations for AHRS
#include "MyCompass.h"
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
float currentHeading, Pitch, Roll; // Variables for storing AHRS data
Adafruit_BNO055 bno = Adafruit_BNO055();

// Declarations for UART com with Arduino NANO to read PWM Values from Radio Receiver
char charArray[10];
String inputString;
String missionCommand;
int winchPulseWidth, rudderPulseWidth;
int windReading, windDirection; //Variables for storing Wind Vane data - Wind Reading corresponds to received angle value (relative to boat), Wind Direction gives actual wind direction (relative to north)
int missionFlag = 0; // 0 if no mission - Remote Mode / 1 if mission is ON - Autonomous Mode
#define nanoSSRXPin 34
#define nanoSSTXPin 35
SoftwareSerial SerialNano(nanoSSRXPin, nanoSSTXPin);

// Declarations for the SD card reader
const int sdCsPin = 2;  // SD card module chip select pin
String dataMessage;
double readLast = 0;
int readFreq = 50; // Sensor Reading frequency in hz
#include "FS.h"
#include "SD.h"
#include "SDReader.h"

// Declarations for Servo Motor Control
int rudder_pwm_min = 0;
int rudder_pwm_max = 0;
int winch_pwm_min = 0;
int winch_pwm_max = 0;
#include "SailServos.h"

// Variables for the Mission and Control Algorithm
int currentWayPoint = 0;
const float latWayPoint[] = {38.690660}; 
const float lngWayPoint[] = {-9.294800};
float desiredHeadingToTarget;
#include "RudderController.h"

void autonomousAction(){
  float distanceToTarget = gps.distanceBetween(currentLat, currentLng, latWayPoint[currentWayPoint], lngWayPoint[currentWayPoint]);
  if (distanceToTarget <= 4 ){
    appendFile(SD, "/tiago.txt", "DESTINATION REACHED\n");
    missionFlag = 0; // Stop mission and return to remote mode
    return;
  }
  float headingError = calculateHeadingError(currentHeading, desiredHeadingToTarget); // Get error between current and desired heading
  float autoRudderAngle = GetRudderControlAction(headingError); // Get best rudder control action given the current heading error
  int autoRudderPulseWidth = ConvertRudderToPWM(autoRudderAngle); // Convert from angle in degrees to corresponding PulseWidth
  SetRudderPosition(autoRudderPulseWidth);
  dataMessage = String(Time) + ", " + String(currentLat, 6) + ", " + String(currentLng, 6) + ", " + String(currentHeading) + ", " + String(desiredHeadingToTarget) + ", " + String(headingError) + ", " + String(Roll) + "," + String(windReading) + "," + String(windDirection) + ", " + String(winchPulseWidth) + ", " + String(autoRudderAngle) + ", " + String(distanceToTarget) + "\r\n";
  appendFile(SD, "/tiago.txt", dataMessage.c_str());
}

void nanoMessageRead(){
  // Takes the received string and separates by commas to assign the values to corresponding variables

  while(SerialNano.available()) { // Go through all messages on the buffer
    inputString = SerialNano.readStringUntil('\n');  // Read the incoming data as string
    // Convert the input string to a char array
    inputString.toCharArray(charArray, 20); // "int winchValue, int rudderValue, char missionCommand, int windReading"
  }
  
  // Find the indices of the commas separating the values
  int comma1Index = inputString.indexOf(',');
  int comma2Index = inputString.indexOf(',' , comma1Index + 1);
  int comma3Index = inputString.indexOf(',', comma2Index + 1);
  if (comma1Index != -1 && comma2Index != -1 && comma3Index != -1) { 
    // Extract the substrings representing the values
    String winchValueStr = inputString.substring(0, comma1Index);
    String rudderValueStr = inputString.substring(comma1Index + 1, comma2Index);
    missionCommand = inputString.substring(comma2Index + 1, comma3Index);
    String windReadingStr = inputString.substring(comma3Index + 1);
    // Convert the substrings to the respective data types
    winchPulseWidth = winchValueStr.toInt();
    rudderPulseWidth = rudderValueStr.toInt();
    int TempWind = windReadingStr.toInt();
    if (TempWind != 0 && TempWind<=360){ // WindVane data sometimes comes wrongly parsed - this filters those outliers 
      windReading = TempWind;
    }
    windDirection = windReading + currentHeading;
    if (windDirection >= 360){
      windDirection -= 360;
    }

    // Check the received pwm values to see if they must update the max/min values for mapping angle-pwm
    if (rudderPulseWidth > rudder_pwm_max && rudderPulseWidth < 2500){
      rudder_pwm_max = rudderPulseWidth;
    }
    if (winchPulseWidth > winch_pwm_max && winchPulseWidth < 2500){
      winch_pwm_max = winchPulseWidth;
    }
    if (rudderPulseWidth < rudder_pwm_min && rudderPulseWidth > 1000 ){
      rudder_pwm_min = rudderPulseWidth;
    }
    if (winchPulseWidth < winch_pwm_min && winchPulseWidth > 1000 ){
      winch_pwm_min = winchPulseWidth;
    }
  }

}

void gpsRead(){
  while (SerialGPS.available() > 0) { // Go through all messages on the buffer
    if (gps.encode(SerialGPS.read()) ) {
      if (gps.location.isValid()){
        float newLat = gps.location.lat();
        float newLng = gps.location.lng();
        float distanceBetweenMeasurements = gps.distanceBetween(currentLat, currentLng, newLat, newLng);
        if ((currentLat== -1.00 && currentLng == -1.00) || distanceBetweenMeasurements <= 10 ) { // Only update measurement if its the first one or if consecutive measurements are not more than 5 meters apart (filter outliers)
          currentLat = newLat;
          currentLng = newLng;
        }
        Speed = gps.speed.knots();
        Course = gps.course.deg();
        Time = gps.time.value();
      }
    }
  }
}

void ahrsRead(){
  // Read AHRS data
  imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  double xmag = mag.x(); double ymag = mag.y(); double zmag = mag.z();
  Pitch = euler.y(); Roll = euler.z();
  currentHeading = HeadingCalc(xmag, ymag, zmag, Pitch, Roll); // Computes heading based on magnetometer readings (calibrates them first) and compensates for tilt
}

void setup() {
  Serial.begin(115200); // Serial monitor

  // SD Card Reader initialization (VSPI)
  pinMode(sdCsPin, OUTPUT);
  initSDCard(sdCsPin);
  // If the file doesn't exist
  // Create a file on the SD card and write the data labels
  File file = SD.open("/tiago.txt");
  if(!file) {
    writeFile(SD, "/tiago.txt", "Time, Latitude, Longitude, currentHeading, desiredHeading, headingError, Roll, WindReading, WindDirection, winchPulseWidth, autoRudderAngle, distanceToTarget \r\n");
  }
  file.close();

  // Initialize Serial interface for com with the Arduino NANO
  SerialNano.begin(9600); // Com with nano

  // GPS Software Serial interface init
  SerialGPS.begin(GPSBaud);
  
  // Compass and AHRS initialization
  while(!bno.begin()) {
    appendFile(SD, "/tiago.txt", "Initializing AHRS \r\n");
    delay(1000);
  }
  delay(2000);
  bno.setExtCrystalUse(true);
  delay(1000);
  // Defines the spatial orientation of the sensor
  bno.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P7);
  bno.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P7);

  // Initialize the PWM Channels for Servo Control
  PWMChanSetup();

  // Build the Rudder Controller
  RudderControllerSetup();
}

void loop() {

  if (millis() - readLast > 1000/readFreq ){

    // Read the last GPS message from ss and get Lat and Lon Values
    // It is only updated at 1Hz
    gpsRead();
    // Get Heading, Roll and Pitch
    ahrsRead();
    // Get Arduino NANO PWM and WindVane Data from Serial interface
    nanoMessageRead(); // Reads the inputString and updates winchPulseWidth, rudderPulseWidth, missionCommand and windvalues

    if (missionCommand == "s" && missionFlag == 0){ // If start command is received and no mission is underway then begin mission
      missionFlag = 1; 
      appendFile(SD, "/tiago.txt", "MISSION START\n");
      desiredHeadingToTarget = gps.courseTo(currentLat, currentLng, latWayPoint[currentWayPoint], lngWayPoint[currentWayPoint]); // desired Heading to get from current position to current waypoint
    }
    else if (missionCommand == "e" && missionFlag == 1){ // If end command is received and mission is underway then stop mission
      missionFlag = 0;
      appendFile(SD, "/tiago.txt", "MISSION END\n");
    }

    if (missionFlag == 0){ // Mission is NOT underway - Remote Mode
      SetRudderPosition(rudderPulseWidth);
      SetWinchPosition(winchPulseWidth);
    }
    if (missionFlag == 1){ // Autonomous Mode is ON      
      autonomousAction();
      SetWinchPosition(winchPulseWidth);
    }

    readLast = millis();
  }
  
}








