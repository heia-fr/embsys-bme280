#pragma once

#include "mbed.h"

class BME280
{
public:
    BME280(PinName sda, PinName scl, char slave_adr);

    int initialize();
    int chipID();
    float getTemperature();
    float getPressure();
    float getHumidity();
  
    static constexpr float kInvalidTempValue = 99.9f;
    static constexpr float kInvalidPressureValue = 9999.0f;
    static constexpr float kInvalidHumValue = -1.0f;

private:
    // private methods
    void internal_init();
  
    // data members
    I2C _bme280;
    uint16_t _chip_id;
    char _address;
    uint16_t _dig_T1;
    int16_t _dig_T2, _dig_T3;
    uint16_t _dig_P1;
    int16_t _dig_P2, _dig_P3, _dig_P4, _dig_P5, _dig_P6, _dig_P7, _dig_P8, _dig_P9;
    uint16_t _dig_H1, _dig_H3;
    int16_t _dig_H2, _dig_H4, _dig_H5, _dig_H6;
    int32_t _t_fine;
    bool _dbg_on;
    
    static constexpr float kMinTemp = -41.0f;
    static constexpr float kMaxTemp = 86.0f;    
    static constexpr float kMinPressure = 300.0f;
    static constexpr float kMaxPressure = 1100.0f;

};