#include <Wire.h>
#include "I2CInterface.h"

class ArduinoI2C : public I2CInterface
{
public:
    void beginTransmission(uint8_t address) override
    {
        Wire.beginTransmission(address);
    }

    uint8_t endTransmission() override
    {
        return Wire.endTransmission();
    }

    uint8_t requestFrom(uint8_t address, uint8_t quantity) override
    {
        return Wire.requestFrom(address, quantity);
    }

    void write(uint8_t data) override
    {
        Wire.write(data);
    }

    uint8_t read() override
    {
        return Wire.read();
    }
};
