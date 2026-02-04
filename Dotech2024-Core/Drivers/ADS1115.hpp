#pragma once
#include "appConfig.hpp"
#ifndef USE_ADS1115
#define USE_ADS1115 0
#endif

#if USE_ADS1115

#include "i2c.h"
namespace ADS1115
{
/**
 * 多路复用器配置 选择 ADS1115 的输入通道配置
 */
typedef enum
{
    CHANNEL_AIN0_AIN1 = 0b000,
    CHANNEL_AIN0_AIN3 = 0b001,
    CHANNEL_AIN1_AIN3 = 0b010,
    CHANNEL_AIN2_AIN3 = 0b011,
    CHANNEL_AIN0_GND  = 0b100,
    CHANNEL_AIN1_GND  = 0b101,
    CHANNEL_AIN2_GND  = 0b110,
    CHANNEL_AIN3_GND  = 0b111
} MultiplexerConfig_t;

/**
 * 可编程增益放大器
 * PGA_6_144 = 0：±6.144 V
 * PGA_4_096：±4.096 V
 * PGA_2_048：±2.048 V
 * PGA_1_024：±1.024 V
 * PGA_0_512：±0.512 V
 * PGA_0_256：±0.256 V
 */
typedef enum
{
    PGA_6_144 = 0,
    PGA_4_096,
    PGA_2_048,
    PGA_1_024,
    PGA_0_512,
    PGA_0_256
} PGA_Config_t;

/**
 * 工作模式
 * 连续转换模式
 * 单次转换模式
 */
typedef enum
{
    MODE_CONTINOUS = 0,
    MODE_SINGLE_SHOT
} OperatingMode_t;

/**
 * 数据采样率
 */
typedef enum
{
    DRATE_8 = 0,
    DRATE_16,
    DRATE_32,
    DRATE_64,
    DRATE_128,
    DRATE_250,
    DRATE_475,
    DRATE_860
} DataRate_t;

/**
 * 比较器模式
 * COMP_HYSTERESIS：单阈值模式，当输入信号超过或低于设定阈值时触发
 * COMP_WINDOW：窗口模式，设置上阈值和下阈值，当输入信号在阈值范围内时触发
 */
typedef enum
{
    COMP_HYSTERESIS = 0,
    COMP_WINDOW
} CompareMode_t;

/**
 * 比较器极性
 * low低电平触发
 * high高电平触发
 */
typedef enum
{
    POLARITY_ACTIVE_LOW = 0,
    POLARITY_ACTIVE_HIGH
} ComparePolarity_t;

/**
 * 锁存模式 none不使用锁存功能
 */
typedef enum
{
    LATCHING_NONE = 0,
    LATCHING_COMPARATOR
} LatchingMode_t;

/**
 * 比较器队列 QUEUE_ONE 在达到阈值条件后，比较器需要连续触发一次才会响应
 */
typedef enum
{
    QUEUE_ONE,
    QUEUE_TWO,
    QUEUE_FOUR,
    QUEUE_DISABLE
} QueueComparator_t;

typedef struct
{
    MultiplexerConfig_t channel;
    PGA_Config_t pgaConfig;
    OperatingMode_t operatingMode;
    DataRate_t dataRate;
    CompareMode_t compareMode;
    ComparePolarity_t polarityMode;
    LatchingMode_t latchingMode;
    QueueComparator_t queueComparator;
} ADS1115_Config_t;

class ADS1115
{
    typedef struct
    {
        uint16_t operationalStatus : 1;
        uint16_t inputMultiplexer : 3;
        uint16_t pgaConfig : 3;
        uint16_t operatingMode : 1;
        uint16_t dataRate : 3;
        uint16_t comparatorMode : 1;
        uint16_t comparatorPolarity : 1;
        uint16_t comparatorLatching : 1;
        uint16_t comparatorQueue : 2;
    } ADS1115_ConfigReg_t;

   public:
    ADS1115(I2C_HandleTypeDef *i2c_handle, uint16_t Addr, ADS1115_Config_t configtemp);

    void ADS1115_updateConfig(ADS1115_Config_t configtemp);
    void ADS1115_updateAddress(uint16_t addresstemp);

    int16_t ADS1115_oneShotMeasure();

    int16_t ADS1115_getData();

    void ADS1115_setThresholds(int16_t lowValue, int16_t highValue);

    void ADS1115_flushData();

    void ADS1115_setConversionReadyPin();

    void ADS1115_startContinousMode();

    void ADS1115_stopContinousMode();

   private:
    I2C_HandleTypeDef *i2c_handle;
    uint16_t address;
    ADS1115_Config_t config;
    void prepareConfigFrame(uint8_t *pOutFrame, ADS1115_Config_t configtemp);
};
}  // namespace ADS1115

#endif