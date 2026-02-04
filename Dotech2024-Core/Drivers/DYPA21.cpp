#include "DYPA21.hpp"
#if USE_DYPA21
#include "FreeRTOS.h"
#include "task.h"
namespace DYPA21
{
DYPA21::DYPA21(I2C_HandleTypeDef *i2c_handle) { this->i2c_handle = i2c_handle; }

int16_t DYPA21::readData()
{
    uint8_t bytes[2]     = {writeRegister, measureCmd};
    uint8_t readBytes[2] = {0};

    if (HAL_I2C_Master_Transmit(this->i2c_handle, this->address, bytes, 2, 100) != HAL_OK)
    {
        return -1;
    }

    vTaskDelay(100);

    if (HAL_I2C_Master_Transmit(this->i2c_handle, this->address, &readDataRegister, 1, 100) != HAL_OK)
    {
        return -1;
    }

    if (HAL_I2C_Master_Receive(this->i2c_handle, this->address, readBytes, 2, 100) != HAL_OK)
    {
        return -1;
    }

    int16_t readValue = (readBytes[0] << 8) | readBytes[1];

    if (readValue == 0)
    {
        HAL_I2C_DeInit(this->i2c_handle);  // 复位I2C总线
        HAL_I2C_Init(this->i2c_handle);    // 重新初始化
        return -1;
    }
    return readValue;
}

int16_t DYPA21::changeAddress(uint8_t oldAddress, uint8_t newAddress)
{
    uint8_t bytes[2] = {addressRegister, newAddress};
    HAL_I2C_Master_Transmit(this->i2c_handle, oldAddress, bytes, 2, 1);
    this->address = newAddress;

    vTaskDelay(1000);
    uint8_t readBytes = 0;
    HAL_I2C_Master_Transmit(this->i2c_handle, newAddress, &addressRegister, 1, 1);
    HAL_I2C_Master_Receive(this->i2c_handle, newAddress, &readBytes, 1, 1);
    int16_t readValue = readBytes;
    return readValue;
}

}  // namespace DYPA21
#endif