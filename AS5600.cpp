#include "AS5600.h"

AS5600::AS5600(I2CInterface *i2c, uint8_t address) : _i2c(i2c), _address(address) {}

bool AS5600::begin(uint8_t directionPin)
{
  _directionPin = directionPin;
  if (_directionPin != AS5600_SW_DIRECTION_PIN)
  {
    pinMode(_directionPin, OUTPUT);
  }
  setDirection(AS5600_CLOCK_WISE);

  return isConnected();
}

bool AS5600::isConnected()
{
  _i2c->beginTransmission(_address);
  return (_i2c->endTransmission() == 0);
}

uint8_t AS5600::getAddress()
{
  return _address;
}

void AS5600::setDirection(uint8_t direction)
{
  _direction = direction;
  if (_directionPin != AS5600_SW_DIRECTION_PIN)
  {
    digitalWrite(_directionPin, _direction);
  }
}

uint8_t AS5600::getDirection()
{
  if (_directionPin != AS5600_SW_DIRECTION_PIN)
  {
    _direction = digitalRead(_directionPin);
  }
  return _direction;
}

uint8_t AS5600::getZMCO()
{
  return readReg(AS5600_ZMCO);
}

bool AS5600::setZPosition(uint16_t value)
{
  if (value > 0x0FFF) return false;
  writeReg2(AS5600_ZPOS, value);
  return true;
}

uint16_t AS5600::getZPosition()
{
  return readReg2(AS5600_ZPOS) & 0x0FFF;
}

bool AS5600::setMPosition(uint16_t value)
{
  if (value > 0x0FFF) return false;
  writeReg2(AS5600_MPOS, value);
  return true;
}

uint16_t AS5600::getMPosition()
{
  return readReg2(AS5600_MPOS) & 0x0FFF;
}

bool AS5600::setMaxAngle(uint16_t value)
{
  if (value > 0x0FFF) return false;
  writeReg2(AS5600_MANG, value);
  return true;
}

uint16_t AS5600::getMaxAngle()
{
  return readReg2(AS5600_MANG) & 0x0FFF;
}

bool AS5600::setConfigure(uint16_t value)
{
  if (value > 0x3FFF) return false;
  writeReg2(AS5600_CONF, value);
  return true;
}

uint16_t AS5600::getConfigure()
{
  return readReg2(AS5600_CONF) & 0x3FFF;
}

bool AS5600::setPowerMode(uint8_t powerMode)
{
  if (powerMode > 3) return false;
  uint8_t value = readReg(AS5600_CONF + 1);
  value &= ~AS5600_CONF_POWER_MODE;
  value |= powerMode;
  writeReg(AS5600_CONF + 1, value);
  return true;
}

uint8_t AS5600::getPowerMode()
{
  return readReg(AS5600_CONF + 1) & 0x03;
}

bool AS5600::setHysteresis(uint8_t hysteresis)
{
  if (hysteresis > 3) return false;
  uint8_t value = readReg(AS5600_CONF + 1);
  value &= ~AS5600_CONF_HYSTERESIS;
  value |= (hysteresis << 2);
  writeReg(AS5600_CONF + 1, value);
  return true;
}

uint8_t AS5600::getHysteresis()
{
  return (readReg(AS5600_CONF + 1) >> 2) & 0x03;
}

bool AS5600::setOutputMode(uint8_t outputMode)
{
  if (outputMode > 2) return false;
  uint8_t value = readReg(AS5600_CONF + 1);
  value &= ~AS5600_CONF_OUTPUT_MODE;
  value |= (outputMode << 4);
  writeReg(AS5600_CONF + 1, value);
  return true;
}

uint8_t AS5600::getOutputMode()
{
  return (readReg(AS5600_CONF + 1) >> 4) & 0x03;
}

bool AS5600::setPWMFrequency(uint8_t pwmFreq)
{
  if (pwmFreq > 3) return false;
  uint8_t value = readReg(AS5600_CONF + 1);
  value &= ~AS5600_CONF_PWM_FREQUENCY;
  value |= (pwmFreq << 6);
  writeReg(AS5600_CONF + 1, value);
  return true;
}

uint8_t AS5600::getPWMFrequency()
{
  return (readReg(AS5600_CONF + 1) >> 6) & 0x03;
}

bool AS5600::setSlowFilter(uint8_t mask)
{
  if (mask > 3) return false;
  uint8_t value = readReg(AS5600_CONF);
  value &= ~AS5600_CONF_SLOW_FILTER;
  value |= mask;
  writeReg(AS5600_CONF, value);
  return true;
}

uint8_t AS5600::getSlowFilter()
{
  return readReg(AS5600_CONF) & 0x03;
}

bool AS5600::setFastFilter(uint8_t mask)
{
  if (mask > 7) return false;
  uint8_t value = readReg(AS5600_CONF);
  value &= ~AS5600_CONF_FAST_FILTER;
  value |= (mask << 2);
  writeReg(AS5600_CONF, value);
  return true;
}

uint8_t AS5600::getFastFilter()
{
  return (readReg(AS5600_CONF) >> 2) & 0x07;
}

bool AS5600::setWatchDog(uint8_t mask)
{
  if (mask > 1) return false;
  uint8_t value = readReg(AS5600_CONF);
  value &= ~AS5600_CONF_WATCH_DOG;
  value |= (mask << 5);
  writeReg(AS5600_CONF, value);
  return true;
}

uint8_t AS5600::getWatchDog()
{
  return (readReg(AS5600_CONF) >> 5) & 0x01;
}

uint16_t AS5600::rawAngle()
{
  int16_t value = readReg2(AS5600_RAW_ANGLE) & 0x0FFF;
  if (_offset > 0) value = (value + _offset) & 0x0FFF;

  if ((_directionPin == AS5600_SW_DIRECTION_PIN) &&
      (_direction == AS5600_COUNTERCLOCK_WISE))
  {
    value = (4096 - value) & 0x0FFF;
  }
  return value;
}

uint16_t AS5600::readAngle()
{
  uint16_t value = readReg2(AS5600_ANGLE) & 0x0FFF;
  if (_offset > 0) value = (value + _offset) & 0x0FFF;

  if ((_directionPin == AS5600_SW_DIRECTION_PIN) &&
      (_direction == AS5600_COUNTERCLOCK_WISE))
  {
    value = (4096 - value) & 0x0FFF;
  }
  return value;
}

bool AS5600::setOffset(float degrees)
{
  if (abs(degrees) > 36000) return false;
  bool neg = (degrees < 0);
  if (neg) degrees = -degrees;

  uint16_t offset = round(degrees * AS5600_DEGREES_TO_RAW);
  offset &= 4095;
  if (neg) offset = 4096 - offset;
  _offset = offset;
  return true;
}

float AS5600::getOffset()
{
  return _offset * AS5600_RAW_TO_DEGREES;
}

bool AS5600::increaseOffset(float degrees)
{
  return setOffset((_offset * AS5600_RAW_TO_DEGREES) + degrees);
}

uint8_t AS5600::readStatus()
{
  return readReg(AS5600_STATUS);
}

uint8_t AS5600::readAGC()
{
  return readReg(AS5600_AGC);
}

uint16_t AS5600::readMagnitude()
{
  return readReg2(AS5600_MAGNITUDE) & 0x0FFF;
}

bool AS5600::detectMagnet()
{
  return (readStatus() & AS5600_MAGNET_DETECT) > 1;
}

bool AS5600::magnetTooStrong()
{
  return (readStatus() & AS5600_MAGNET_HIGH) > 1;
}

bool AS5600::magnetTooWeak()
{
  return (readStatus() & AS5600_MAGNET_LOW) > 1;
}

float AS5600::getAngularSpeed(uint8_t mode)
{
  uint32_t now     = micros();
  int      angle   = readAngle();
  uint32_t deltaT  = now - _lastMeasurement;
  int      deltaA  = angle - _lastAngle;

  if (deltaA >  2048) deltaA -= 4096;
  if (deltaA < -2048) deltaA += 4096;
  float    speed   = (deltaA * 1e6) / deltaT;

  _lastMeasurement = now;
  _lastAngle       = angle;

  if (mode == AS5600_MODE_RADIANS)
  {
    return speed * AS5600_RAW_TO_RADIANS;
  }
  if (mode == AS5600_MODE_RPM)
  {
    return speed * AS5600_RAW_TO_RPM;
  }
  return speed * AS5600_RAW_TO_DEGREES;
}

int32_t AS5600::getCumulativePosition()
{
  int16_t value = readReg2(AS5600_ANGLE) & 0x0FFF;
  if (_error != AS5600_OK) return _position;

  if ((_lastPosition > 2048) && (value < (_lastPosition - 2048)))
  {
    _position = _position + 4096 - _lastPosition + value;
  }
  else if ((value > 2048) && (_lastPosition < (value - 2048)))
  {
    _position = _position - 4096 - _lastPosition + value;
  }
  else _position = _position - _lastPosition + value;
  _lastPosition = value;

  return _position;
}

int32_t AS5600::getRevolutions()
{
  int32_t p = _position >> 12;
  return p;
}

int32_t AS5600::resetPosition(int32_t position)
{
  int32_t old = _position;
  _position = position;
  return old;
}

int32_t AS5600::resetCumulativePosition(int32_t position)
{
  _lastPosition = readReg2(AS5600_RAW_ANGLE) & 0x0FFF;
  int32_t old = _position;
  _position = position;
  return old;
}

int AS5600::lastError()
{
  int value = _error;
  _error = AS5600_OK;
  return value;
}

uint8_t AS5600::readReg(uint8_t reg)
{
  _error = AS5600_OK;
  _i2c->beginTransmission(_address);
  _i2c->write(reg);
  if (_i2c->endTransmission() != 0)
  {
    _error = AS5600_ERROR_I2C_READ_0;
    return 0;
  }
  uint8_t n = _i2c->requestFrom(_address, (uint8_t)1);
  if (n != 1)
  {
    _error = AS5600_ERROR_I2C_READ_1;
    return 0;
  }
  return _i2c->read();
}

uint16_t AS5600::readReg2(uint8_t reg)
{
  _error = AS5600_OK;
  _i2c->beginTransmission(_address);
  _i2c->write(reg);
  if (_i2c->endTransmission() != 0)
  {
    _error = AS5600_ERROR_I2C_READ_2;
    return 0;
  }
  uint8_t n = _i2c->requestFrom(_address, (uint8_t)2);
  if (n != 2)
  {
    _error = AS5600_ERROR_I2C_READ_3;
    return 0;
  }
  uint16_t value = _i2c->read();
  value <<= 8;
  value |= _i2c->read();
  return value;
}

uint8_t AS5600::writeReg(uint8_t reg, uint8_t value)
{
  _error = AS5600_OK;
  _i2c->beginTransmission(_address);
  _i2c->write(reg);
  _i2c->write(value);
  if (_i2c->endTransmission() != 0)
  {
    _error = AS5600_ERROR_I2C_WRITE_0;
  }
  return _error;
}

uint8_t AS5600::writeReg2(uint8_t reg, uint16_t value)
{
  _error = AS5600_OK;
  _i2c->beginTransmission(_address);
  _i2c->write(reg);
  _i2c->write(value >> 8);
  _i2c->write(value & 0xFF);
  if (_i2c->endTransmission() != 0)
  {
    _error = AS5600_ERROR_I2C_WRITE_0;
  }
  return _error;
}

AS5600L::AS5600L(uint8_t address, I2CInterface *i2c) : AS5600(i2c, address) {}

bool AS5600L::setAddress(uint8_t address)
{
  if ((address < 8) || (address > 119)) return false;

  writeReg(AS5600_I2CADDR, address << 1);
  writeReg(AS5600_I2CUPDT, address << 1);

  _address = address;
  return true;
}

bool AS5600L::setI2CUPDT(uint8_t value)
{
  if ((value < 8) || (value > 119)) return false;
  writeReg(AS5600_I2CUPDT, value << 1);
  return true;
}

uint8_t AS5600L::getI2CUPDT()
{
  return (readReg(AS5600_I2CUPDT) >> 1);
}
