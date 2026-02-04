#pragma once
#include "appConfig.hpp"
#ifndef USE_DYPA21
#define USE_DYPA21 0
#endif

#if USE_DYPA21

#include "i2c.h"
namespace DYPA21
{
class DYPA21
{
   public:
    DYPA21(I2C_HandleTypeDef *i2c_handle);

    int16_t readData();

    int16_t changeAddress(uint8_t oldAddress, uint8_t newAddress);

   private:
    I2C_HandleTypeDef *i2c_handle;
    int address              = 0xE8;
    uint8_t writeRegister    = 0x10;
    uint8_t measureCmd       = 0xBD;
    uint8_t readDataRegister = 0x02;
    uint8_t addressRegister  = 0x05;
};
}  // namespace DYPA21
#endif