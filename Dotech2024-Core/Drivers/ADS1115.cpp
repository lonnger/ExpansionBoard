#include "ADS1115.hpp"

#if USE_ADS1115

#include "FreeRTOS.h"
#include "task.h"
namespace ADS1115
{
ADS1115::ADS1115(I2C_HandleTypeDef *i2c_handle, uint16_t Addr, ADS1115_Config_t configtemp)
{
    this->i2c_handle = i2c_handle;
    this->address    = Addr;
    this->config     = configtemp;
}
void ADS1115::prepareConfigFrame(uint8_t *pOutFrame, ADS1115_Config_t configtemp)
{
    pOutFrame[0] = 0x01;
    pOutFrame[1] |= (configtemp.channel << 4) | (configtemp.pgaConfig << 1) | (configtemp.operatingMode << 0);
    pOutFrame[2] |=
        (configtemp.dataRate << 5) | (configtemp.compareMode << 4) | (configtemp.polarityMode << 3) | (configtemp.latchingMode << 2) | (configtemp.queueComparator << 0);
}
void ADS1115::ADS1115_updateConfig(ADS1115_Config_t configtemp)
{
    this->config = configtemp;

    uint8_t bytes[3] = {0};
    prepareConfigFrame(bytes, this->config);

    HAL_I2C_Master_Transmit(this->i2c_handle, (this->address << 1), bytes, 3, 1);
    vTaskDelay(3);
}

void ADS1115::ADS1115_updateAddress(uint16_t addresstemp) { this->address = addresstemp; }

int16_t ADS1115::ADS1115_oneShotMeasure()
{
    uint8_t bytes[3] = {0};

    prepareConfigFrame(bytes, this->config);

    bytes[1] |= (1 << 7);  // OS one shot measure

    HAL_I2C_Master_Transmit(this->i2c_handle, (this->address << 1), bytes, 3, 1);

    return ADS1115_getData();
}

int16_t ADS1115::ADS1115_getData()
{
    uint8_t bytes[2] = {0};
    bytes[0]         = 0x00;
    HAL_I2C_Master_Transmit(this->i2c_handle, (this->address << 1), bytes, 1, 1);

    if (HAL_I2C_Master_Receive(this->i2c_handle, (this->address << 1), bytes, 2, 1) != HAL_OK)
        return 0;

    int16_t readValue = ((bytes[0] << 8) | bytes[1]);
    if (readValue < 0)
        readValue = 0;

    return readValue;
}

void ADS1115::ADS1115_setThresholds(int16_t lowValue, int16_t highValue)
{
    uint8_t ADSWrite[3] = {0};

    // hi threshold reg
    ADSWrite[0] = 0x03;
    ADSWrite[1] = (uint8_t)((highValue & 0xFF00) >> 8);
    ADSWrite[2] = (uint8_t)(highValue & 0x00FF);
    HAL_I2C_Master_Transmit(this->i2c_handle, (this->address << 1), ADSWrite, 3, 1);

    // lo threshold reg
    ADSWrite[0] = 0x02;
    ADSWrite[1] = (uint8_t)((lowValue & 0xFF00) >> 8);
    ADSWrite[2] = (uint8_t)(lowValue & 0x00FF);
    HAL_I2C_Master_Transmit(this->i2c_handle, (this->address << 1), ADSWrite, 3, 1);
}

void ADS1115::ADS1115_flushData() { ADS1115_getData(); }

void ADS1115::ADS1115_setConversionReadyPin() { ADS1115_setThresholds(0x0000, 0xFFFF); }

void ADS1115::ADS1115_startContinousMode()
{
    uint8_t bytes[3] = {0};

    ADS1115_Config_t configReg = this->config;
    configReg.operatingMode    = MODE_CONTINOUS;
    prepareConfigFrame(bytes, configReg);

    HAL_I2C_Master_Transmit(this->i2c_handle, (this->address << 1), bytes, 3, 1);
}

void ADS1115::ADS1115_stopContinousMode()
{
    uint8_t bytes[3]           = {0};
    ADS1115_Config_t configReg = this->config;
    configReg.operatingMode    = MODE_SINGLE_SHOT;
    prepareConfigFrame(bytes, configReg);

    HAL_I2C_Master_Transmit(this->i2c_handle, (this->address << 1), bytes, 3, 1);
}
}  // namespace ADS1115

#endif