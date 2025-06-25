#pragma once

#include <Arduino.h>

enum moveDirection
{
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
};

enum encoderType
{
    LEFT_ENCODER,
    RIGHT_ENCODER,
};

class Platform
{
  private:
    const int leftDirectionMotorPin = 18;
    const int rightDirectionMotorPin = 19;
    const int leftMotorPin = 23;
    const int rightMotorPin = 5;

    const int leftMotorEncoderPinA = 34;
    const int leftMotorEncoderPinB = 35;
    const int rightMotorEncoderPinA = 39;
    const int rightMotorEncoderPinB = 36;

    const int freq = 5000;         // Frecuencia en Hz
    const int pwmChannelLeft = 1;  // Canal 1 para pin 23
    const int pwmChannelRight = 2; // Canal 0 para pin 5
    const int resolution = 8;      // Resolución: 8 bits (0-255)

    int32_t posLeft = 0;
    int32_t posRight = 0;
    int32_t lastPosLeft = 0;
    int32_t lastPosRight = 0;

    int lastStateLeftA = 0;
    int lastStateRightA = 0;

  public:
    Platform(/* args */);
    ~Platform();

    std::function<void(const int32_t, const int32_t)> onEncodersValues;

    void moveWheels(moveDirection direction, uint8_t speed);
    void stopWheels();
    void readEncoders();
    void emitEncodersValues(const int32_t &leftEnc, const int32_t &rightEnc);
};

Platform::Platform(/* args */)
{
    pinMode(leftDirectionMotorPin, OUTPUT);  // Left Direction
    pinMode(rightDirectionMotorPin, OUTPUT); // Right Direction
    pinMode(leftMotorPin, OUTPUT);           // Lidar Motor
    pinMode(rightMotorPin, OUTPUT);          // Lidar Motor

    pinMode(leftMotorEncoderPinA, INPUT);
    pinMode(leftMotorEncoderPinB, INPUT);
    pinMode(rightMotorEncoderPinA, INPUT);
    pinMode(rightMotorEncoderPinB, INPUT);

    ledcSetup(pwmChannelRight, freq, resolution);
    ledcAttachPin(rightMotorPin, pwmChannelRight);

    ledcSetup(pwmChannelLeft, freq, resolution);
    ledcAttachPin(leftMotorPin, pwmChannelLeft);

    lastStateLeftA = digitalRead(leftMotorEncoderPinA);
    lastStateRightA = digitalRead(rightMotorEncoderPinA);
}

Platform::~Platform()
{
}

void Platform::emitEncodersValues(const int32_t &leftEnc, const int32_t &rightEnc)
{
    if (onEncodersValues)
    {
        onEncodersValues(leftEnc, rightEnc);
    }
}

void Platform::moveWheels(moveDirection direction, uint8_t speed)
{
    switch (direction)
    {
    case FORWARD:
        digitalWrite(leftDirectionMotorPin, HIGH);
        digitalWrite(rightDirectionMotorPin, HIGH);
        ledcWrite(pwmChannelLeft, speed);
        ledcWrite(pwmChannelRight, speed);
        break;
    case BACKWARD:
        digitalWrite(leftDirectionMotorPin, LOW);
        digitalWrite(rightDirectionMotorPin, LOW);
        ledcWrite(pwmChannelLeft, speed);
        ledcWrite(pwmChannelRight, speed);
        break;
    case LEFT:
        digitalWrite(leftDirectionMotorPin, HIGH);
        digitalWrite(rightDirectionMotorPin, LOW);
        ledcWrite(pwmChannelLeft, speed);
        ledcWrite(pwmChannelRight, speed);
        break;
    case RIGHT:
        digitalWrite(leftDirectionMotorPin, LOW);
        digitalWrite(rightDirectionMotorPin, HIGH);
        ledcWrite(pwmChannelLeft, speed);
        ledcWrite(pwmChannelRight, speed);
        break;
    }
}

void Platform::stopWheels()
{
    digitalWrite(leftDirectionMotorPin, LOW);
    digitalWrite(rightDirectionMotorPin, LOW);
    ledcWrite(pwmChannelLeft, 0);
    ledcWrite(pwmChannelRight, 0);
}

void Platform::readEncoders()
{

    // === LEFT ENCODER ===
    int stateLeftA = digitalRead(leftMotorEncoderPinA);
    if (stateLeftA != lastStateLeftA)
    {
        int stateLeftB = digitalRead(leftMotorEncoderPinB);
        if (stateLeftA == stateLeftB)
        {
            posLeft++;
        }
        else
        {
            posLeft--;
        }
    }
    lastStateLeftA = stateLeftA;

    // === RIGHT ENCODER ===
    int stateRightA = digitalRead(rightMotorEncoderPinA);
    if (stateRightA != lastStateRightA)
    {
        int stateRightB = digitalRead(rightMotorEncoderPinB);
        if (stateRightA == stateRightB)
        {
            posRight++;
        }
        else
        {
            posRight--;
        }
    }

    lastStateRightA = stateRightA;

    if (posLeft != lastPosLeft || posRight != lastPosRight)
    {
        emitEncodersValues(posLeft, posRight);
        lastPosLeft = posLeft;
        lastPosRight = posRight;
    }

    delay(1);
}