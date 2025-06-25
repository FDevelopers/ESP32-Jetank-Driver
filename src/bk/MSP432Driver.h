#include <ArduinoJson.h>
#include <HardwareSerial.h>
#include <cstdint>

struct TI_RSLK
{
    uint32_t leftAEncoder;
    uint32_t leftBEncoder;
    uint32_t rightBEncoder;
    uint32_t rightAEncoder;
    uint8_t batteryLevel;
};

class MSP432Driver
{
  private:
    int messageLength;
    String Transmitcommand;
    std::function<void(TI_RSLK myRobot)> listener;
    void emitReceive(TI_RSLK myRobot)
    {
        if (listener)
        {
            listener(myRobot);
        }
    }

    // Preprarar Funcion evento que se ejecute cuando reciba informacion de los sensores de TI-RSLK

  public:
    MSP432Driver(/* args */);
    ~MSP432Driver();
    void ReadSensors(HardwareSerial *mspSerial);
    void writeCommand(HardwareSerial *mspSerial, String command);
    void onReceive(std::function<void(TI_RSLK myRobot)> callback)
    {
        listener = callback;
    }
};

MSP432Driver::MSP432Driver(/* args */)
{
}

MSP432Driver::~MSP432Driver()
{
}

void MSP432Driver::writeCommand(HardwareSerial *mspSerial, String command)
{
    Serial.printf("commandT: %s\n", command.c_str());
    mspSerial->print(command);
}

void MSP432Driver::ReadSensors(HardwareSerial *mspSerial)
{
    
    if (mspSerial->available())
    {
        String dataReceive;
        while (mspSerial->available())
        {
            dataReceive += mspSerial->read();
        }

        if (dataReceive.length() > 0)
        {
            TI_RSLK myRobot;
            JsonDocument doc;
            deserializeJson(doc, dataReceive);

            myRobot.leftAEncoder = doc["LA"];
            myRobot.leftBEncoder = doc["LB"];
            myRobot.rightAEncoder = doc["RA"];
            myRobot.rightBEncoder = doc["RB"];
            myRobot.batteryLevel = doc["battery"];
        }
    }
}