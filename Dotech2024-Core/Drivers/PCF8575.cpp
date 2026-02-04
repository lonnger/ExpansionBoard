#include "PCF8575.hpp"
#if USE_PCF8575
#include "FreeRTOS.h"
#include "task.h"
namespace PCF8575
{
PCF8575::PCF8575(I2C_HandleTypeDef *i2c_handle) { this->i2c_handle = i2c_handle; }
int16_t PCF8575::readBytes()
{
    uint8_t bytes[2] = {0};
    if (HAL_I2C_Master_Receive(this->i2c_handle, this->readAddress, bytes, 2, 1) != HAL_OK)
        return 0;
    int16_t readValue = ((bytes[1] << 8) | bytes[0]);
    return readValue;
}

void PCF8575::writeBytes(int16_t data)
{
    uint8_t bytes[2] = {0};
    bytes[0]         = data;
    bytes[1]         = data >> 8;
    HAL_I2C_Master_Transmit(this->i2c_handle, this->writeAddress, bytes, 2, 1);
}

uint8_t PCF8575::readBit(uint8_t bit)
{
    uint8_t data;
    data = PCF8575::readBytes();
    return (data & (1 << bit));
}

void PCF8575::writeBit(uint8_t bit, uint8_t sta)
{
    uint8_t data;
    data = PCF8575::readBytes();
    if (sta == 0)
        data &= ~(1 << bit);
    else
        data |= 1 << bit;
    PCF8575::writeBytes(data);  // 写入新的数据
}

}  // namespace PCF8575
#endif