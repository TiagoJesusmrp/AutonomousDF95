#ifndef OTA_H
#define OTA_H
#include "Arduino.h"
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <ESPAsyncWebSrv.h>

void setupOTA(void);

void handleOTA(void);

#endif