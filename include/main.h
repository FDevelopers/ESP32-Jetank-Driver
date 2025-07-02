#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoWebsockets.h>
#include <HardwareSerial.h>
#include <RPLidar.h>
#include "MPU9250.h"
#include <WiFi.h>

#include "components/tcpClient.h"
#include "components/platform.h"

#define RPLIDAR_MOTOR 0
#define STATUS_LED 4

using namespace websockets;

const char *ssid = "Fd_Network";
const char *password = "fd10-pd12-dg06";
const char *tcp_host = "10.0.0.206";
const uint16_t tcp_port = 8000;



void SensorTask(void *pvParameters);
void LidarTask(void *pvParameters);
void setup_wifi();
void WheelCommand(uint8_t command);
void calibrateIMU(uint8_t cmd);
void encodersCallback(const int32_t &leftEnc, const int32_t &rightEnc);
void setupOTA();