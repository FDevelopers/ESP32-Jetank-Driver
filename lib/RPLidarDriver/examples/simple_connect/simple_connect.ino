
#include <RPLidar.h>

RPLidar lidar;

#define RPLIDAR_MOTOR 23


void setup() {
  Serial.begin(115200);

  lidar.begin(Serial1);
  pinMode(RPLIDAR_MOTOR, OUTPUT);

  Serial.println("setup");
}

void loop() {
  // if (IS_OK(lidar.waitPoint())) {
  //   float distance = lidar.getCurrentPoint().distance;  //distance value in mm unit
  //   float angle = lidar.getCurrentPoint().angle;        //anglue value in degree
  //   bool startBit = lidar.getCurrentPoint().startBit;   //whether this point is belong to a new scan
  //   byte quality = lidar.getCurrentPoint().quality;     //quality of the current measurement
  // }

  Serial.println("test");
  delay(100);
  // } else {
  //   analogWrite(RPLIDAR_MOTOR, 0);  //stop the rplidar motor
  //   rplidar_response_device_info_t info;
  //   if (IS_OK(lidar.getDeviceInfo(info, 100))) {
  //     lidar.startScan();
  //     analogWrite(RPLIDAR_MOTOR, 255);
  //     delay(1000);
  //   }
  // }
}
