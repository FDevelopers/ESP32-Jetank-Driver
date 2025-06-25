#include <Arduino.h>

class Acelerometer
{
  private:
    int xpin = 34;
    int ypin = 35;
    int zpin = 32;

  public:
    Acelerometer(int x, int y, int z);
    ~Acelerometer();

    float ReadAxisX();
    float ReadAxisY();
    float ReadAxisZ();


};

Acelerometer::Acelerometer(int x, int y, int z) : xpin(x), ypin(y), zpin(z)
{
}

Acelerometer::~Acelerometer()
{
}

float Acelerometer::ReadAxisX()
{
    return ((analogRead(xpin) * 3.3 / 1023.0) - 1.65) / 0.3;
}

float Acelerometer::ReadAxisY()
{
    return ((analogRead(ypin) * 3.3 / 1023.0) - 1.65) / 0.3;
}

float Acelerometer::ReadAxisZ()
{
    return ((analogRead(zpin) * 3.3 / 1023.0) - 1.65) / 0.3;
}