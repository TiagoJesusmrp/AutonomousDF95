/*
TIAGO JESUS
AUTONOMOUS DF95 MODEL SAILBOAT


SAVE TAB SEPARATED MAGNETOMETER READINGS TO SD CARD
THE SAVED FILE IS THEN GIVEN TO MAGNETO SOFTWARE TO GET CALIBRATION VALUES

Sensor must be moved accross the 3D plane

*/


#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "FS.h"
#include "SD.h"
const int sdCsPin = 2;  // SD card module chip select pin

// Create accelerometer/magnetometer object
Adafruit_BNO055 bno = Adafruit_BNO055();


// Initialize SD card
void initSDCard(int SD_CS_PIN){
   if (!SD.begin(SD_CS_PIN)){ // SD card CS Pin as 17
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE){
    Serial.println("No SD card attached");
    return;
  }
  Serial.print("SD Card Type: ");
  if(cardType == CARD_MMC){
    Serial.println("MMC");
  } else if(cardType == CARD_SD){
    Serial.println("SDSC");
  } else if(cardType == CARD_SDHC){
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
}

// Write to the SD card
void writeFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if(file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

// Append data to the SD card
void appendFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if(file.print(message)) {
    //Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}


void setup() {
  Serial.begin(115200);  // Baud rate

  // SD Card Reader initialization (VSPI)
  pinMode(sdCsPin, OUTPUT);
  initSDCard(sdCsPin);
  // If the file doesn't exist
  // Create a file on the SD card and write the data labels
  File file = SD.open("/Mag.txt");
  if(!file) {
    writeFile(SD, "/Mag.txt", "X, Y, Z\r\n");
  }
  file.close();


  while(!bno.begin()) {
    delay(500);
  }
  delay(2000);
  bno.setExtCrystalUse(true);
  delay(1000);
  // Defines the spatial orientation of the sensor
  bno.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P7);
  bno.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P7);

  delay(500);
}



void loop()
{
    imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    // Save to SD
    String dataMsg = String(mag.x()) + "	" + String(mag.y()) + "        " + String(mag.z()) + "\n";
    delay(100);  // 10 Hz              
}

