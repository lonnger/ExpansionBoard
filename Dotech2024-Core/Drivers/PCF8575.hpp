#pragma once
#include "appConfig.hpp"
#ifndef USE_PCF8575
#define USE_PCF8575 0
#endif

#if USE_PCF8575

#include "i2c.h"
namespace PCF8575
{
class PCF8575
{
   public:
    PCF8575(I2C_HandleTypeDef *i2c_handle);
    int16_t readBytes();
    void  writeBytes(int16_t data);
    void writeBit(uint8_t bit,uint8_t sta);
    uint8_t readBit(uint8_t bit);


   private:
    I2C_HandleTypeDef *i2c_handle;
    int writeAddress = 0x40;
    int readAddress = 0x41;
};
}  // namespace PCF8575
#endif