#include "main.h"
#include <ArduinoOTA.h>
#include <ESPmDNS.h>

TaskHandle_t taskHandleIMU = NULL;
TaskHandle_t taskHandleLidar = NULL;

HardwareSerial LIDARSerial(1);
RPLidar lidar;
MPU9250 IMU(Wire, 0x68);

Platform plataforma;
TcpClient tcpClient;

void connectToWiFi()
{
    WiFi.config(IPAddress(10, 0, 0, 68), IPAddress(255, 255, 255, 0), IPAddress(10, 0, 0, 1), IPAddress(8, 8, 8, 8));
    WiFi.begin(ssid, password);
    Serial.print("Conectando a WiFi");
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.printf("✅ WiFi conectado a %s\n", WiFi.localIP().toString().c_str());
}

void TaskIMU(void *pvParameters)
{
    float imuData[11];
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50); // 20Hz
    uint8_t ledState = LOW;

    lidar.begin(LIDARSerial);
    if (IMU.begin() < 0)
    {
        Serial.println("❌ IMU no se pudo inicializar");
        vTaskDelete(NULL);
    }

    IMU.setAccelRange(MPU9250::ACCEL_RANGE_2G);
    IMU.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
    IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);

    while (true)
    {
        if (!tcpClient.isConnected())
        {
            tcpClient.connectToServer();
        }
        else
        {
            IMU.readSensor();
            imuData[0] = IMU.getAccelX_mss();
            imuData[1] = IMU.getAccelY_mss();
            imuData[2] = IMU.getAccelZ_mss();
            imuData[3] = IMU.getGyroX_rads();
            imuData[4] = IMU.getGyroY_rads();
            imuData[5] = IMU.getGyroZ_rads();
            imuData[6] = IMU.getMagX_uT();
            imuData[7] = IMU.getMagY_uT();
            imuData[8] = IMU.getMagZ_uT();
            imuData[9] = IMU.getTemperature_C();
            imuData[10] = millis() / 1000.0;
            tcpClient.sendPacket(0x01, (uint8_t *)imuData, sizeof(imuData));
            digitalWrite(4, ledState);
            ledState = !ledState;
            tcpClient.receivePacket();
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }
    }
}

void TaskLidar(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // 10Hz
    float lidarData[3];

    ledcSetup(0, 1000, 8);
    ledcAttachPin(RPLIDAR_MOTOR, 0);
    ledcWrite(0, 255); // Enciende motor
    lidar.startScan(true);

    while (true)
    {
        if (IS_OK(lidar.waitPoint()))
        {
            lidarData[0] = lidar.getCurrentPoint().distance;
            lidarData[1] = lidar.getCurrentPoint().angle;
            lidarData[2] = (float)lidar.getCurrentPoint().quality;
            tcpClient.sendPacket(0x02, (uint8_t *)lidarData, sizeof(lidarData));
        }
        else
        {
            ledcWrite(0, 0); // Apagar motor
            rplidar_response_device_info_t info;
            if (IS_OK(lidar.getDeviceInfo(info, 100)))
            {
                lidar.startScan();
                ledcWrite(0, 255);
                delay(1000);
            }
        }

        plataforma.readEncoders();
        // vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void setup()
{
    Serial.begin(115200);
    LIDARSerial.begin(115200, SERIAL_8N1, 16, 17);

    pinMode(RPLIDAR_MOTOR, OUTPUT); // Lidar Motor
    pinMode(STATUS_LED, OUTPUT);    // Status LED

    plataforma.stopWheels();
    plataforma.onEncodersValues = encodersCallback;

    connectToWiFi();
    setupOTA();

    tcpClient.connectToServer();
    tcpClient.onMotorCommand = [](uint8_t cmd) { WheelCommand(cmd); };
    tcpClient.onIMUCommand = [](uint8_t cmd) { calibrateIMU(cmd); };

    xTaskCreatePinnedToCore(TaskIMU, "IMU Task", 4096, NULL, 1, &taskHandleIMU, 0);
    xTaskCreatePinnedToCore(TaskLidar, "Lidar Task", 4096, NULL, 1, &taskHandleLidar, 1);
}

void loop()
{
    ArduinoOTA.handle(); // ← permite OTA desde loop
}

void WheelCommand(uint8_t command)
{
    switch (command)
    {
    case 1:
        Serial.println("📦 Foward ");
        plataforma.moveWheels(FORWARD, 64);
        break;
    case 2:
        Serial.println("📦 Backward ");
        plataforma.moveWheels(BACKWARD, 64);
        break;
    case 3:
        Serial.println("📦 Left ");
        plataforma.moveWheels(LEFT, 64);
        break;
    case 4:
        Serial.println("📦 Right ");
        plataforma.moveWheels(RIGHT, 64);
        break;
    case 5:
        Serial.println("📦 Stop ");
        plataforma.stopWheels();
        break;
    default:
        Serial.print("❓ Comando desconocido: ");
        Serial.println(command);
        break;
    }
}

void calibrateIMU(uint8_t cmd)
{
    switch (cmd)
    {
    case 1:
        Serial.println("📦 Calibrando Acelerometro ");
        IMU.calibrateAccel();
        Serial.println("✅ Fin de Calibración");
        break;
    case 2:
        Serial.println("📦 Calibrando Magnetometro ");
        IMU.calibrateMag();
        Serial.println("✅ Fin de Calibración");
        break;
    case 3:
        Serial.println("📦 Calibrando Gyro ");
        IMU.calibrateGyro();
        Serial.println("✅ Fin de Calibración");
        break;
    default:
        Serial.println("❌ Comando desconocido");
        break;
    }
}

void encodersCallback(const int32_t &leftEnc, const int32_t &rightEnc)
{
    int32_t encoders[2] = {leftEnc, rightEnc};
    tcpClient.sendPacket(0x03, (uint8_t *)encoders, sizeof(encoders));

    // int ppr = 250;
    // float wheelDiameter = 6.5f;
    // float perimeter = wheelDiameter * 3.14159265359f;

    // float leftDistance = leftEnc * perimeter / ppr;
    // float rightDistance = rightEnc * perimeter / ppr;

    // Serial.printf("📦 Distance left: %f, right: %f\n", leftDistance, rightDistance);
}

void setupOTA()
{
    // ArduinoOTA.setHostname("esp32-ota");

    ArduinoOTA.onStart([]() {
        Serial.printf("✅ Starting OTA %s update.\n", ArduinoOTA.getCommand() == U_FLASH ? "sketch" : "filesystem");
    });

    ArduinoOTA.onEnd([]() {
        Serial.println("✅ OTA Done");
        ESP.restart();
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("📦 OTA Progress: %u%%\n", (progress * 100) / total);
    });

    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("❌ OTA Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR)
            Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR)
            Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR)
            Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR)
            Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR)
            Serial.println("End Failed");
    });

    ArduinoOTA.begin();
}