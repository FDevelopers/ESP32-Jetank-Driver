#include <Arduino.h>
#include <WiFi.h>
#include <functional>
#include <string>

#include "./components/packet.h"
#include "platform.h"

class TcpClient
{
  private:
    WiFiClient tcpClient;
    SemaphoreHandle_t sendMutex;

    const char *tcp_host = "10.0.0.206";
    const uint16_t tcp_port = 8000;
    void emitMotorCommand(const uint8_t &data);
    void emitIMUCommand(const uint8_t &data);

  public:
    TcpClient();
    ~TcpClient();

    std::function<void(const uint8_t)> onMotorCommand;
    std::function<void(const uint8_t)> onIMUCommand;

    bool connectToServer();
    void sendPacket(uint8_t packetId, const uint8_t *payload, uint16_t payloadSize);
    void receivePacket();
    bool isConnected();
    void handleReceivedPacket(uint8_t packetId, const uint8_t *payload, uint16_t size);
};

TcpClient::TcpClient()
{
    sendMutex = xSemaphoreCreateMutex();
}

TcpClient::~TcpClient()
{
}

void TcpClient::emitMotorCommand(const uint8_t &data)
{
    if (onMotorCommand)
    {
        onMotorCommand(data); // Llama al callback
    }
}

void TcpClient::emitIMUCommand(const uint8_t &data)
{
    if (onIMUCommand)
    {
        onIMUCommand(data); // Llama al callback
    }
}

bool TcpClient::connectToServer()
{
    if (!tcpClient.connected())
    {
        Serial.println("🔄 Intentando conectar al servidor TCP...");
        if (tcpClient.connect(tcp_host, tcp_port))
        {
            tcpClient.setNoDelay(true);
            Serial.println("✅ Conectado al servidor TCP");
            return true;
        }
        Serial.println("❌ Falló la conexión TCP");
        return false;
    }
    return true;
}

void TcpClient::sendPacket(uint8_t packetId, const uint8_t *payload, uint16_t payloadSize)
{
    if (!connectToServer())
        return;

    Packet packet(packetId, payload, payloadSize);
    uint8_t *data = packet.getPacket();
    uint16_t size = packet.getPacketSize();

    xSemaphoreTake(sendMutex, portMAX_DELAY);
    tcpClient.write(data, size);
    xSemaphoreGive(sendMutex);
    
    // xSemaphoreTake(sendMutex, portMAX_DELAY);
    // tcpClient.write(header, 4);
    // tcpClient.write(payload, payloadSize);
    // tcpClient.write(&checksum, 1);
    // xSemaphoreGive(sendMutex);
}

void TcpClient::receivePacket()
{
    // Verifica si hay datos disponibles en el buffer TCP
    while (tcpClient.available() > 0)
    {
        static enum
        {
            WAIT_START,
            READ_HEADER,
            READ_PAYLOAD,
            READ_CHECKSUM
        } state = WAIT_START;

        static uint8_t header[4];
        static uint16_t payloadSize = 0;
        static uint8_t payload[512]; // Aumenta si tu payload es mayor
        static uint8_t index = 0;
        static uint8_t checksum = 0;

        uint8_t incomingByte = tcpClient.read();

        switch (state)
        {
        case WAIT_START:
            if (incomingByte == 0xAA)
            {
                header[0] = incomingByte;
                index = 1;
                checksum = incomingByte;
                state = READ_HEADER;
            }
            break;

        case READ_HEADER:
            header[index] = incomingByte;
            checksum ^= incomingByte;
            index++;
            if (index == 4)
            {
                payloadSize = header[2] | (header[3] << 8);
                if (payloadSize > sizeof(payload))
                {
                    Serial.println("❌ Payload demasiado grande");
                    state = WAIT_START;
                    break;
                }
                index = 0;
                state = READ_PAYLOAD;
            }
            break;

        case READ_PAYLOAD:
            payload[index] = incomingByte;
            checksum ^= incomingByte;
            index++;
            if (index == payloadSize)
            {
                index = 0;
                state = READ_CHECKSUM;
            }
            break;

        case READ_CHECKSUM:
            if (incomingByte == checksum)
            {
                uint8_t packetId = header[1];
                handleReceivedPacket(packetId, payload, payloadSize);
            }
            else
            {
                Serial.println("❌ Checksum inválido");
            }
            state = WAIT_START;
            break;
        }
    }
}

bool TcpClient::isConnected()
{
    return tcpClient.connected();
}

void TcpClient::handleReceivedPacket(uint8_t packetId, const uint8_t *payload, uint16_t size)
{
    switch (packetId)
    {
    case 0x01:
        Serial.print("📦 Comandos Control Motores.. ");
        emitMotorCommand(payload[0]);
        break;
    case 0x02:
        Serial.println("📦 Comando Calibracion IMU ");
        emitIMUCommand(payload[0]);
        break;
    default:
        Serial.print("❓ Paquete desconocido ID: ");
        Serial.println(packetId, HEX);
        break;
    }
}
