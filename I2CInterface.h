#ifndef I2CINTERFACE_H
#define I2CINTERFACE_H

class I2CInterface
{
public:
    virtual ~I2CInterface() {}
    virtual void beginTransmission(uint8_t address) = 0;
    virtual uint8_t endTransmission() = 0;
    virtual uint8_t requestFrom(uint8_t address, uint8_t quantity) = 0;
    virtual void write(uint8_t data) = 0;
    virtual uint8_t read() = 0;
};

#endif // I2CINTERFACE_H
