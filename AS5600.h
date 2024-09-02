#pragma once
//
//    FILE: AS5600.h
//  AUTHOR: Rob Tillaart
// VERSION: 0.6.1
// PURPOSE: Arduino library for AS5600 magnetic rotation meter
//    DATE: 2022-05-28
//     URL: https://github.com/RobTillaart/AS5600

#include "Arduino.h"
#include "I2CInterface.h"  // 引入I2C接口头文件

#define AS5600_LIB_VERSION              (F("0.6.1"))

//  default addresses
const uint8_t AS5600_DEFAULT_ADDRESS    = 0x36;
const uint8_t AS5600L_DEFAULT_ADDRESS   = 0x40;
const uint8_t AS5600_SW_DIRECTION_PIN   = 255;

//  setDirection
const uint8_t AS5600_CLOCK_WISE         = 0;  //  LOW
const uint8_t AS5600_COUNTERCLOCK_WISE  = 1;  //  HIGH

//  0.087890625;
const float   AS5600_RAW_TO_DEGREES     = 360.0 / 4096;
const float   AS5600_DEGREES_TO_RAW     = 4096 / 360.0;
//  0.00153398078788564122971808758949;
const float   AS5600_RAW_TO_RADIANS     = PI * 2.0 / 4096;
//  4.06901041666666e-6
const float   AS5600_RAW_TO_RPM         = 60.0 / 4096;

//  getAngularSpeed
const uint8_t AS5600_MODE_DEGREES       = 0;
const uint8_t AS5600_MODE_RADIANS       = 1;
const uint8_t AS5600_MODE_RPM           = 2;

//  ERROR CODES
const int     AS5600_OK                 = 0;
const int     AS5600_ERROR_I2C_READ_0   = -100;
const int     AS5600_ERROR_I2C_READ_1   = -101;
const int     AS5600_ERROR_I2C_READ_2   = -102;
const int     AS5600_ERROR_I2C_READ_3   = -103;
const int     AS5600_ERROR_I2C_WRITE_0  = -200;
const int     AS5600_ERROR_I2C_WRITE_1  = -201;

// 配置寄存器地址和位掩码
const uint8_t AS5600_ZMCO               = 0x00;
const uint8_t AS5600_ZPOS               = 0x01;   //  + 0x02
const uint8_t AS5600_MPOS               = 0x03;   //  + 0x04
const uint8_t AS5600_MANG               = 0x05;   //  + 0x06
const uint8_t AS5600_CONF               = 0x07;   //  + 0x08

const uint8_t AS5600_CONF_POWER_MODE    = 0x03;
const uint8_t AS5600_CONF_HYSTERESIS    = 0x0C;
const uint8_t AS5600_CONF_OUTPUT_MODE   = 0x30;
const uint8_t AS5600_CONF_PWM_FREQUENCY = 0xC0;
const uint8_t AS5600_CONF_SLOW_FILTER   = 0x03;
const uint8_t AS5600_CONF_FAST_FILTER   = 0x1C;
const uint8_t AS5600_CONF_WATCH_DOG     = 0x20;

// 输出寄存器地址
const uint8_t AS5600_RAW_ANGLE          = 0x0C;   //  + 0x0D
const uint8_t AS5600_ANGLE              = 0x0E;   //  + 0x0F

// I2C 地址寄存器（AS5600L）
const uint8_t AS5600_I2CADDR            = 0x20;
const uint8_t AS5600_I2CUPDT            = 0x21;

// 状态寄存器地址
const uint8_t AS5600_STATUS             = 0x0B;
const uint8_t AS5600_AGC                = 0x1A;
const uint8_t AS5600_MAGNITUDE          = 0x1B;   //  + 0x1C
const uint8_t AS5600_BURN               = 0xFF;

// 状态位掩码
const uint8_t AS5600_MAGNET_HIGH        = 0x08;
const uint8_t AS5600_MAGNET_LOW         = 0x10;
const uint8_t AS5600_MAGNET_DETECT      = 0x20;

// 配置常量
// setOutputMode
const uint8_t AS5600_OUTMODE_ANALOG_100 = 0;
const uint8_t AS5600_OUTMODE_ANALOG_90  = 1;
const uint8_t AS5600_OUTMODE_PWM        = 2;

// setPowerMode
const uint8_t AS5600_POWERMODE_NOMINAL  = 0;
const uint8_t AS5600_POWERMODE_LOW1     = 1;
const uint8_t AS5600_POWERMODE_LOW2     = 2;
const uint8_t AS5600_POWERMODE_LOW3     = 3;

// setPWMFrequency
const uint8_t AS5600_PWM_115            = 0;
const uint8_t AS5600_PWM_230            = 1;
const uint8_t AS5600_PWM_460            = 2;
const uint8_t AS5600_PWM_920            = 3;

// setHysteresis
const uint8_t AS5600_HYST_OFF           = 0;
const uint8_t AS5600_HYST_LSB1          = 1;
const uint8_t AS5600_HYST_LSB2          = 2;
const uint8_t AS5600_HYST_LSB3          = 3;

// setSlowFilter
const uint8_t AS5600_SLOW_FILT_16X      = 0;
const uint8_t AS5600_SLOW_FILT_8X       = 1;
const uint8_t AS5600_SLOW_FILT_4X       = 2;
const uint8_t AS5600_SLOW_FILT_2X       = 3;

// setFastFilter
const uint8_t AS5600_FAST_FILT_NONE     = 0;
const uint8_t AS5600_FAST_FILT_LSB6     = 1;
const uint8_t AS5600_FAST_FILT_LSB7     = 2;
const uint8_t AS5600_FAST_FILT_LSB9     = 3;
const uint8_t AS5600_FAST_FILT_LSB18    = 4;
const uint8_t AS5600_FAST_FILT_LSB21    = 5;
const uint8_t AS5600_FAST_FILT_LSB24    = 6;
const uint8_t AS5600_FAST_FILT_LSB10    = 7;

// setWatchDog
const uint8_t AS5600_WATCHDOG_OFF       = 0;
const uint8_t AS5600_WATCHDOG_ON        = 1;

class AS5600
{
public:
  AS5600(I2CInterface *i2c, uint8_t address = AS5600_DEFAULT_ADDRESS);

  bool     begin(uint8_t directionPin = AS5600_SW_DIRECTION_PIN);
  bool     isConnected();

  uint8_t  getAddress();

  void     setDirection(uint8_t direction = AS5600_CLOCK_WISE);
  uint8_t  getDirection();

  uint8_t  getZMCO();

  bool     setZPosition(uint16_t value);
  uint16_t getZPosition();

  bool     setMPosition(uint16_t value);
  uint16_t getMPosition();

  bool     setMaxAngle(uint16_t value);
  uint16_t getMaxAngle();

  bool     setConfigure(uint16_t value);
  uint16_t getConfigure();

  bool     setPowerMode(uint8_t powerMode);
  uint8_t  getPowerMode();

  bool     setHysteresis(uint8_t hysteresis);
  uint8_t  getHysteresis();

  bool     setOutputMode(uint8_t outputMode);
  uint8_t  getOutputMode();

  bool     setPWMFrequency(uint8_t pwmFreq);
  uint8_t  getPWMFrequency();

  bool     setSlowFilter(uint8_t mask);
  uint8_t  getSlowFilter();

  bool     setFastFilter(uint8_t mask);
  uint8_t  getFastFilter();

  bool     setWatchDog(uint8_t mask);
  uint8_t  getWatchDog();

  uint16_t rawAngle();
  uint16_t readAngle();

  bool     setOffset(float degrees);
  float    getOffset();
  bool     increaseOffset(float degrees);

  uint8_t  readStatus();
  uint8_t  readAGC();
  uint16_t readMagnitude();

  bool     detectMagnet();
  bool     magnetTooStrong();
  bool     magnetTooWeak();

  float    getAngularSpeed(uint8_t mode = AS5600_MODE_DEGREES);

  int32_t  getCumulativePosition();
  int32_t  getRevolutions();
  int32_t  resetPosition(int32_t position = 0);
  int32_t  resetCumulativePosition(int32_t position = 0);

  int      lastError();

protected:
  uint8_t  readReg(uint8_t reg);
  uint16_t readReg2(uint8_t reg);
  uint8_t  writeReg(uint8_t reg, uint8_t value);
  uint8_t  writeReg2(uint8_t reg, uint16_t value);

  I2CInterface* _i2c;
  uint8_t  _address         = AS5600_DEFAULT_ADDRESS;
  uint8_t  _directionPin    = 255;
  uint8_t  _direction       = AS5600_CLOCK_WISE;
  int      _error           = AS5600_OK;

  uint32_t _lastMeasurement = 0;
  int16_t  _lastAngle       = 0;

  uint16_t _offset          = 0;

  int32_t  _position        = 0;
  int16_t  _lastPosition    = 0;
};

/////////////////////////////////////////////////////////////////////////////
//
// AS5600L
//
class AS5600L : public AS5600
{
public:
  AS5600L(uint8_t address = AS5600L_DEFAULT_ADDRESS, I2CInterface *i2c = nullptr);

  bool     setAddress(uint8_t address);

  bool     setI2CUPDT(uint8_t value);
  uint8_t  getI2CUPDT();
};

//  -- END OF FILE --
