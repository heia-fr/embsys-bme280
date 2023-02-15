#include "bme280.hpp"

#include "mbed_trace.h"
#if defined(MBED_CONF_MBED_TRACE_ENABLE)
#define TRACE_GROUP  "BME280"
#endif // MBED_CONF_MBED_TRACE_ENABLE

BME280::BME280(PinName sda, PinName scl, char slave_adr) 
  : _bme280(sda, scl) 
{       
    _address = slave_adr;
    _bme280.frequency(100000);
}
    
void BME280::internal_init()
{
    char cmd[18] = {0};
    wait_us(5000);  
    
    //tr_debug("\033[0m\033[2J\033[H ++++ BME-P register's ++++\r\n");
     
    // ctrl_hum
    cmd[0] = 0xF2;
    // Humidity oversampling x1
    cmd[1] = 0x01;                  
    _bme280.write(_address, cmd, 2);
 
    // ctrl_meas
    cmd[0] = 0xF4;
    // Temperature oversampling x1, Pressure oversampling x1, Normal mode                  
    cmd[1] = 0x27;
    _bme280.write(_address, cmd, 2);
 
    // config
    cmd[0] = 0xF5;
    // Standby 1000ms, Filter off
    cmd[1] = 0xa0;   
    _bme280.write(_address, cmd, 2);
    
    // sensor registers
    //tr_debug("chip_id = 0x%x\n\n", _chip_id);
     
    // read dig_T calibration regs
    cmd[0] = 0x88;                  
    _bme280.write(_address, cmd, 1);
    _bme280.read(_address, cmd, 6); 
    _dig_T1 = (cmd[1] << 8) | cmd[0];
    _dig_T2 = (cmd[3] << 8) | cmd[2];
    _dig_T3 = (cmd[5] << 8) | cmd[4]; 
    //tr_debug("Temp Cal reg's:\nT1 = 0x%x\nT2 = 0x%x\nT3 = 0x%x\n", _dig_T1, _dig_T2, _dig_T3);
    
    // read dig_P calibration regs
    cmd[0] = 0x8E;                  
    _bme280.write(_address, cmd, 1);
    _bme280.read(_address, cmd, 18); 
    _dig_P1 = (cmd[ 1] << 8) | cmd[ 0];
    _dig_P2 = (cmd[ 3] << 8) | cmd[ 2];
    _dig_P3 = (cmd[ 5] << 8) | cmd[ 4];
    _dig_P4 = (cmd[ 7] << 8) | cmd[ 6];
    _dig_P5 = (cmd[ 9] << 8) | cmd[ 8];
    _dig_P6 = (cmd[11] << 8) | cmd[10];
    _dig_P7 = (cmd[13] << 8) | cmd[12];
    _dig_P8 = (cmd[15] << 8) | cmd[14];
    _dig_P9 = (cmd[17] << 8) | cmd[16];    
    //tr_debug("Pressure Cal reg's:\nP1 = 0x%x\nP2 = 0x%x\nP3 = 0x%x\nP4 = 0x%x", _dig_P1, _dig_P2, _dig_P3, _dig_P4);
    //tr_debug("P5 = 0x%x\nP6 = 0x%x\nP7 = 0x%x\nP8 = 0x%x\nP9 = 0x%x\n", _dig_P5, _dig_P6, _dig_P7, _dig_P8, _dig_P9);
    
    if (_chip_id == 0x60) {   
        // Only BME280 has Humidity 
        // read dig_H calibration LSB regs             
        cmd[0] = 0xA1;              
        _bme280.write(_address, cmd, 1);
        _bme280.read(_address, cmd, 1);
        // read dig_H calibration MSB regs
        cmd[1] = 0xE1;              
        _bme280.write(_address, &cmd[1], 1);
        _bme280.read(_address, &cmd[1], 7);
        _dig_H1 = cmd[0];
        _dig_H2 = (cmd[2] << 8) | cmd[1];
        _dig_H3 = cmd[3];
        _dig_H4 = (cmd[4] << 4) | (cmd[5] & 0x0f);
        _dig_H5 = (cmd[6] << 4) | ((cmd[5]>>4) & 0x0f);
        _dig_H6 = cmd[7];    
        //tr_debug("Humidity Cal reg's:\nH1 = 0x%x\nH2 = 0x%x\nH3 = 0x%x", _dig_H1, _dig_H2, _dig_H3);
        //tr_debug("H4 = 0x%x\nH5 = 0x%x\nH6 = 0x%x\n", _dig_H4, _dig_H5, _dig_H6);
    }
} 

int BME280::initialize()
{
    char cmd[2] = {0};
    // reset reg
    cmd[0] = 0xE0;      
    cmd[1] = 0xB6;
    _bme280.write(_address, cmd, 2);

    if (chipID() != 0) {
        internal_init();
        return _chip_id;
    }
    else {
        return 0;
    }
}

int BME280::chipID()
{
    char cmd[1] = {0};
    // chip_id
    cmd[0] = 0xD0;      
    _bme280.write(_address, cmd, 1);
    cmd[0] = 0x00;
    _bme280.read(_address, cmd, 1);
    _chip_id = cmd[0];

    return _chip_id;
}

float BME280::getTemperature()
{
    if (chipID() == 0) {      
      // check if sensor is present
      if (initialize() == 0) {
        return kInvalidTempValue;
      }
    }     
        
    char cmd[4] = {0};
    // temp_msb
    cmd[0] = 0xFA;      
    _bme280.write(_address, cmd, 1);
    _bme280.read(_address, &cmd[1], 3);
    
    int32_t adc_T = (cmd[1] << 12) | (cmd[2] << 4) | (cmd[3] >> 4);
        
    int32_t var1  = ((((adc_T>>3) - ((int32_t) _dig_T1 <<1))) * ((int32_t) _dig_T2)) >> 11;    
    int32_t var2  = (((((adc_T>>4) - ((int32_t) _dig_T1)) * ((adc_T>>4) - ((int32_t) _dig_T1))) >> 12) * ((int32_t) _dig_T3)) >> 14;    
    _t_fine = var1 + var2;    
    int32_t T  = (_t_fine * 5 + 128) >> 8;
    float temp = T/100.0;    
    // return temperature if within device limits.
    if (temp> kMinTemp && temp < kMaxTemp) {    
        return temp;
    }
    else {
      // error value     
      return kInvalidTempValue; 
    }
}
 
float BME280::getPressure()
{
    if (chipID() == 0) {      
      // check if sensor is present
      if (initialize() == 0) {
        return kInvalidPressureValue;
      }
    }

    char cmd[4] = {0}; 
    // press_msb
    cmd[0] = 0xF7;      
    _bme280.write(_address, cmd, 1);
    _bme280.read(_address, &cmd[1], 3);
     
    uint32_t adc_P = (cmd[1] << 12) | (cmd[2] << 4) | (cmd[3] >> 4);
     
    int64_t var1 = ((int64_t) _t_fine) - 128000;
    int64_t var2 = var1 * var1 * (int64_t) _dig_P6;
    var2 = var2 + ((var1 * (int64_t) _dig_P5) << 17);
    var2 = var2 + (((int64_t) _dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t) _dig_P3)>>8)+((var1 * (int64_t) _dig_P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1)) * ((int64_t) _dig_P1)>>33;
    if (var1 == 0) {
      return kInvalidPressureValue;
    }
    
    int64_t p = 1048576 - adc_P;
    p = (((p<<31)-var2)*3125)/var1;
    var1 = (((int64_t) _dig_P9) * (p>>13) * (p>>13))>>25;
    var2 = (((int64_t) _dig_P8) * p)>>19;
    p = ((p + var1 + var2)>>8) + (((int64_t) _dig_P7)<<4);
    float press = ((float) p /256.0)/100.0f;    
    if (press > kMinPressure && press < kMaxPressure) {    
        // return temperature if within device limits.
        return press;
    }
    else {
      // error value
      return kInvalidPressureValue;
    }
}
 
float BME280::getHumidity()
{    
    if (chipID() == 0) {      
      // check if sensor is present
      if (initialize() == 0) {
        return kInvalidHumValue;
      }
    }

    char cmd[4] = {0};
    // hum_msb
    cmd[0] = 0xfd; 
    _bme280.write(_address, cmd, 1);
    _bme280.read(_address, &cmd[1], 2);
 
    uint32_t humid_raw = (cmd[1] << 8) | cmd[2];
 
    int32_t v_x1r = (_t_fine - 76800);
    v_x1r = (((((humid_raw << 14) -(((int32_t) _dig_H4) << 20) - (((int32_t) _dig_H5) *
            v_x1r)) + ((int32_t)16384)) >> 15) * (((((((v_x1r *
            (int32_t) _dig_H6) >> 10) * (((v_x1r * ((int32_t) _dig_H3)) >> 11) +
            32768)) >> 10) + 2097152) * (int32_t) _dig_H2 + 8192) >> 14));
    v_x1r = (v_x1r - (((((v_x1r >> 15) * (v_x1r >> 15)) >> 7) *
            (int32_t) _dig_H1) >> 4));
    v_x1r = (v_x1r < 0 ? 0 : v_x1r);
    v_x1r = (v_x1r > 419430400 ? 419430400 : v_x1r);
 
    float humid = ((float)(v_x1r >> 12))/1024.0f;
 
    return humid;
}