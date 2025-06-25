// #include "main.h"
// #include "MPU9250.h"
// #include "RPLidar.h"
// #include <HardwareSerial.h>
// #include <WiFi.h>

// HardwareSerial LIDARSerial(1);

// WiFiClient tcpClient;
// RPLidar lidar;
// MPU9250 IMU(Wire, 0x68);

// SemaphoreHandle_t sendMutex;

// void connectToWiFi()
// {
//     WiFi.begin(ssid, password);
//     Serial.print("Conectando a WiFi");
//     while (WiFi.status() != WL_CONNECTED)
//     {
//         delay(500);
//         Serial.print(".");
//     }
//     Serial.println("✅ WiFi conectado");
// }

// bool connectToServer()
// {
//     if (!tcpClient.connected())
//     {
//         Serial.println("🔄 Intentando conectar al servidor TCP...");
//         if (tcpClient.connect(tcp_host, tcp_port))
//         {
//             tcpClient.setNoDelay(true);
//             Serial.println("✅ Conectado al servidor TCP");
//             return true;
//         }
//         Serial.println("❌ Falló la conexión TCP");
//         return false;
//     }
//     return true;
// }

// void sendPacket(uint8_t packetId, const uint8_t *payload, uint16_t payloadSize)
// {
//     if (!connectToServer())
//         return;

//     uint8_t header[4];
//     header[0] = 0xAA; // Start byte
//     header[1] = packetId;
//     header[2] = payloadSize & 0xFF;
//     header[3] = (payloadSize >> 8) & 0xFF;

//     uint8_t checksum = 0;
//     for (int i = 0; i < 4; ++i)
//         checksum ^= header[i];
//     for (int i = 0; i < payloadSize; ++i)
//         checksum ^= payload[i];

//     xSemaphoreTake(sendMutex, portMAX_DELAY);
//     tcpClient.write(header, 4);
//     tcpClient.write(payload, payloadSize);
//     tcpClient.write(&checksum, 1);
//     xSemaphoreGive(sendMutex);
// }

// void TaskIMU(void *pvParameters)
// {
//     float imuData[11];
//     TickType_t xLastWakeTime = xTaskGetTickCount();
//     const TickType_t xFrequency = pdMS_TO_TICKS(50); // 20Hz
//     uint8_t ledState = LOW;

//     while (true)
//     {
//         if (!tcpClient.connected())
//         {
//             connectToServer();
//         }
//         else
//         {
//             IMU.readSensor();
//             imuData[0] = IMU.getAccelX_mss();
//             imuData[1] = IMU.getAccelY_mss();
//             imuData[2] = IMU.getAccelZ_mss();
//             imuData[3] = IMU.getGyroX_rads();
//             imuData[4] = IMU.getGyroY_rads();
//             imuData[5] = IMU.getGyroZ_rads();
//             imuData[6] = IMU.getMagX_uT();
//             imuData[7] = IMU.getMagY_uT();
//             imuData[8] = IMU.getMagZ_uT();
//             imuData[9] = IMU.getTemperature_C();
//             imuData[10] = millis() / 1000.0;
//             sendPacket(0x01, (uint8_t *)imuData, sizeof(imuData));
//             digitalWrite(4, ledState);
//             ledState = !ledState;
//             receivePacket();
//             vTaskDelayUntil(&xLastWakeTime, xFrequency);
//         }
//     }
// }

// void TaskLidar(void *pvParameters)
// {
//     TickType_t xLastWakeTime = xTaskGetTickCount();
//     const TickType_t xFrequency = pdMS_TO_TICKS(100); // 10Hz
//     float lidarData[3];

//     ledcSetup(0, 1000, 8);
//     ledcAttachPin(RPLIDAR_MOTOR, 0);
//     ledcWrite(0, 255); // Enciende motor
//     lidar.startScan(true);

//     while (true)
//     {
//         if (IS_OK(lidar.waitPoint()))
//         {
//             lidarData[0] = lidar.getCurrentPoint().distance;
//             lidarData[1] = lidar.getCurrentPoint().angle;
//             lidarData[2] = (float)lidar.getCurrentPoint().quality;
//             sendPacket(0x02, (uint8_t *)lidarData, sizeof(lidarData));
//         }
//         else
//         {
//             ledcWrite(0, 0); // Apagar motor
//             rplidar_response_device_info_t info;
//             if (IS_OK(lidar.getDeviceInfo(info, 100)))
//             {
//                 lidar.startScan();
//                 ledcWrite(0, 255);
//                 delay(1000);
//             }
//         }
//         // vTaskDelayUntil(&xLastWakeTime, xFrequency);
//     }
// }

// void setup()
// {
//     Serial.begin(115200);
//     LIDARSerial.begin(115200, SERIAL_8N1, 16, 17);

//     pinMode(LEFT_DIRECTION, OUTPUT);  // Left Direction
//     pinMode(RIGHT_DIRECTION, OUTPUT); // Right Direction

//     pinMode(RPLIDAR_MOTOR, OUTPUT); // Lidar Motor
//     pinMode(STATUS_LED, OUTPUT);    // Status LED

//     ledcSetup(pwmChannelRight, freq, resolution);
//     ledcAttachPin(RIGHT_MOTOR, pwmChannelRight);

//     ledcSetup(pwmChannelLeft, freq, resolution);
//     ledcAttachPin(LEFT_MOTOR, pwmChannelLeft);

//     lidar.begin(LIDARSerial);
//     if (IMU.begin() < 0)
//     {
//         Serial.println("❌ IMU no se pudo inicializar");
//         while (1)
//             ;
//     }

//     IMU.setAccelRange(MPU9250::ACCEL_RANGE_2G);
//     IMU.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
//     IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);

//     connectToWiFi();
//     connectToServer();
//     sendMutex = xSemaphoreCreateMutex();

//     xTaskCreatePinnedToCore(TaskIMU, "IMU Task", 4096, NULL, 1, NULL, 0);
//     xTaskCreatePinnedToCore(TaskLidar, "Lidar Task", 4096, NULL, 1, NULL, 1);
// }

// void loop()
// {
//     vTaskDelete(NULL);
// }

// void receivePacket()
// {
//     // Verifica si hay datos disponibles en el buffer TCP
//     while (tcpClient.available() > 0)
//     {
//         static enum
//         {
//             WAIT_START,
//             READ_HEADER,
//             READ_PAYLOAD,
//             READ_CHECKSUM
//         } state = WAIT_START;

//         static uint8_t header[4];
//         static uint16_t payloadSize = 0;
//         static uint8_t payload[512]; // Aumenta si tu payload es mayor
//         static uint8_t index = 0;
//         static uint8_t checksum = 0;

//         uint8_t incomingByte = tcpClient.read();

//         switch (state)
//         {
//         case WAIT_START:
//             if (incomingByte == 0xAA)
//             {
//                 header[0] = incomingByte;
//                 index = 1;
//                 checksum = incomingByte;
//                 state = READ_HEADER;
//             }
//             break;

//         case READ_HEADER:
//             header[index] = incomingByte;
//             checksum ^= incomingByte;
//             index++;
//             if (index == 4)
//             {
//                 payloadSize = header[2] | (header[3] << 8);
//                 if (payloadSize > sizeof(payload))
//                 {
//                     Serial.println("❌ Payload demasiado grande");
//                     state = WAIT_START;
//                     break;
//                 }
//                 index = 0;
//                 state = READ_PAYLOAD;
//             }
//             break;

//         case READ_PAYLOAD:
//             payload[index] = incomingByte;
//             checksum ^= incomingByte;
//             index++;
//             if (index == payloadSize)
//             {
//                 index = 0;
//                 state = READ_CHECKSUM;
//             }
//             break;

//         case READ_CHECKSUM:
//             if (incomingByte == checksum)
//             {
//                 uint8_t packetId = header[1];
//                 handleReceivedPacket(packetId, payload, payloadSize);
//             }
//             else
//             {
//                 Serial.println("❌ Checksum inválido");
//             }
//             state = WAIT_START;
//             break;
//         }
//     }
// }

// void handleReceivedPacket(uint8_t packetId, const uint8_t *payload, uint16_t size)
// {
//     switch (packetId)
//     {
//     case 0x01:
//         Serial.print("📦 Comandos Control Motores.. ");
//         provideCommand(payload[0]);
//         break;
//     case 0x02:
//         Serial.println("📦 Comando Calibracion IMU ");
//         calibrateIMU(payload[0]);
//         break;
//     default:
//         Serial.print("❓ Paquete desconocido ID: ");
//         Serial.println(packetId, HEX);
//         break;
//     }
// }

// void provideCommand(char command)
// {
//     switch (command)
//     {
//     case '1':
//         Serial.println("📦 Foward ");
//         digitalWrite(LEFT_DIRECTION, HIGH);
//         digitalWrite(RIGHT_DIRECTION, HIGH);
//         ledcWrite(pwmChannelLeft, 64);
//         ledcWrite(pwmChannelRight, 64);
//         break;
//     case '2':
//         Serial.println("📦 Backward ");
//         digitalWrite(LEFT_DIRECTION, LOW);
//         digitalWrite(RIGHT_DIRECTION, LOW);
//         ledcWrite(pwmChannelLeft, 64);
//         ledcWrite(pwmChannelRight, 64);
//         break;
//     case '3':
//         Serial.println("📦 Left ");
//         digitalWrite(LEFT_DIRECTION, HIGH);
//         digitalWrite(RIGHT_DIRECTION, LOW);
//         ledcWrite(pwmChannelLeft, 64);
//         ledcWrite(pwmChannelRight, 64);
//         break;
//     case '4':
//         Serial.println("📦 Right ");
//         digitalWrite(LEFT_DIRECTION, LOW);
//         digitalWrite(RIGHT_DIRECTION, HIGH);
//         ledcWrite(pwmChannelLeft, 64);
//         ledcWrite(pwmChannelRight, 64);
//         break;
//     case '5':
//         Serial.println("📦 Stop ");
//         ledcWrite(pwmChannelLeft, 0);
//         ledcWrite(pwmChannelRight, 0);
//         break;
//     default:
//         Serial.print("❓ Comando desconocido: ");
//         Serial.println(command);
//         break;
//     }
// }

// void calibrateIMU(int cmd)
// {
//     switch (cmd)
//     {
//     case 1:
//         Serial.println("📦 Calibrando Acelerometro ");
//         IMU.calibrateAccel();
//         Serial.println("✅ Fin de Calibración");
//         break;
//     case 2:
//         Serial.println("📦 Calibrando Magnetometro ");
//         IMU.calibrateMag();
//         Serial.println("✅ Fin de Calibración");
//         break;
//     case 3:
//         Serial.println("📦 Calibrando Gyro ");
//         IMU.calibrateGyro();
//         Serial.println("✅ Fin de Calibración");
//         break;
//     default:
//         Serial.println("❌ Comando desconocido");
//         break;
//     }
// }
