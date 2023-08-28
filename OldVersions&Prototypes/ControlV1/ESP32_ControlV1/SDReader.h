#ifndef SDREADER_H
#define SDREADER_H
#include <Arduino.h>
#include "FS.h"
#include "SD.h"


void initSDCard(int SD_CS_PIN);
void writeFile(fs::FS &fs, const char * path, const char * message);
void appendFile(fs::FS &fs, const char * path, const char * message);


#endif