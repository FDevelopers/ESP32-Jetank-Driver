#include <Arduino.h>

class Packet
{
  private:
    uint8_t *packet;
    uint16_t packetSize;

  public:
    Packet(uint8_t packetId, const uint8_t *payload, uint16_t payloadSize);
    ~Packet();

    uint8_t *getPacket();
    uint16_t getPacketSize();
};

Packet::Packet(uint8_t packetId, const uint8_t *payload, uint16_t payloadSize)
{
    packetSize = payloadSize + 5;
    packet = new uint8_t[packetSize];
    if (packet == nullptr)
    {
        Serial.println("Error: Memory allocation failed for packet!");
        return;
    }

    packet[0] = 0xAA;
    packet[1] = packetId;
    packet[2] = payloadSize & 0xFF;
    packet[3] = (payloadSize >> 8) & 0xFF;

    for (uint16_t i = 0; i < payloadSize; ++i)
    {
        packet[i + 4] = payload[i];
    }

    uint8_t checksum = 0;
    for (uint16_t i = 0; i < packetSize - 1; ++i)
    {
        checksum ^= packet[i];
    }
    packet[packetSize - 1] = checksum;
}

Packet::~Packet()
{
    delete[] packet;
    packet = nullptr;
}

uint8_t *Packet::getPacket()
{
    return packet;
}

uint16_t Packet::getPacketSize()
{
    return packetSize;
}