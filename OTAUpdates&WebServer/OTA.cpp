#include "OTA.h"
#include "Arduino.h"
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <ESPAsyncWebSrv.h>

/*
TIAGO JESUS
AUTONOMOUS DF95 MODEL SAILBOAT

This is the library created for enabling OTA updates for the ESP32 
and creating the webserver to access the SD card contents.

Change networks credentials in the code and then access files through
any device connected to the same network using following "links":

http://ESP32IPADDRESS/list?path=/file.txt   to list all files on SD
http://ESP32IPADDRESS/file?path=/file.txt   to get file.txt from SD
http://ESP32IPADDRESS/delete?path=/file.txt to delete file.txt from SD

OTA updates are done directly through Arduino IDE by selecting the OTA port.

*/



#define SD_CS_PIN 2
AsyncWebServer server(80);
// Insert used network credentials here
const char* ssid = "Redmi";
const char* password = "";
bool wifiConnected = false;


void setupOTA() { // Setups OTA and SD card server
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  const unsigned long WIFI_TIMEOUT_MS = 5000;  // 5 seconds

  unsigned long startAttemptTime = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < WIFI_TIMEOUT_MS) {
    Serial.print(".");
    delay(100);
  }
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    ArduinoOTA.begin();
  
  
  // Route for getting file list
  server.on("/list", HTTP_GET, [](AsyncWebServerRequest *request){
    String path = "/";
    Serial.println("List: " + path);
    File root = SD.open(path);
    path = String();

    String output = "[";
    if(root){
      File file = root.openNextFile();
      while(file){
        if (output != "[") {
          output += ',';
        }
        output += "{\"type\":\"";
        output += (file.isDirectory()) ? "dir" : "file";
        output += "\",\"name\":\"";
        output += String(file.name()).substring(1);
        output += "\"}";
        file = root.openNextFile();
      }
    }
    output += "]";
    request->send(200, "text/json", output);
    root.close();
  });

  // Route for downloading a file
  server.on("/file", HTTP_GET, [](AsyncWebServerRequest *request){
    String path = request->urlDecode(request->arg("path"));
    Serial.println("Download: " + path);
    if(SD.exists(path)){
      request->send(SD, path, "application/octet-stream");
    } else {
      request->send(404);
    }
  });

  // Route for deleting a file 
  server.on("/delete", HTTP_GET, [](AsyncWebServerRequest *request){
  if (request->hasArg("path")) {
    String path = request->urlDecode(request->arg("path"));
    Serial.println("Delete: " + path);
    if (SD.exists(path)) {
      if (SD.remove(path)) {
        request->send(200, "text/plain", "File deleted");
      } else {
        request->send(500, "text/plain", "Delete failed");
      }
    } else {
      request->send(404, "text/plain", "File not found");
    }
  } else {
    request->send(400, "text/plain", "Bad request");
  }
});

  server.begin();
  }
  Serial.println("Ready for OTA updates");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void handleOTA() {
  if (wifiConnected) {
    ArduinoOTA.handle();
  }
}