/**
 * @file DM4310.hpp
 * @author cooper Wang
 * @brief
 * @version 0.2
 * @date 2023-02-21
 *
 * @copyright
 *
 */

#pragma once
#include "appConfig.hpp"
#ifndef USE_DM4310
#define USE_DM4310 0
#endif

#if USE_DM4310 && defined(HAL_CAN_MODULE_ENABLED)
#include "main.h"
#include "stdint.h"

#define STD_ID 0x01
#define MASTER_ID 0x00
namespace DM4310
{
enum MotorMode : uint8_t
{
    MIT,
    TRI_Loop,
    SPEED
};

enum MotorERR : uint8_t
{
    NO_ERR = 7,
    OVER_VOLTAGE,
    UNDER_VOLTAGE,
    OVER_CURRENT,
    MOS_OVER_HEAT,
    ROTOR_OVER_HEAT,
    LOSE_CONNECTION,
    OVER_LOAD
};

class DM4310
{
   public:
    DM4310(const DM4310 &)            = delete;
    DM4310 &operator=(const DM4310 &) = delete;

    /*
     * @brief enable the motor
     */
    void enableMotor();

    /*
     * @brief disable the motor
     */
    void disableMotor();

    /**
     * @brief Get the raw encoder value
     *
     * @return uint16_t
     */
    uint16_t getRawEncoder() const;

    /**
     * @brief Get the current position of the motor in radian
     * @note  You may need to multiply the reduction ratio of the motor to get the actual position.
     *
     * @return float
     */
    virtual float getPosition() const;

    /**
     * @brief Set the Current Position object in radian
     * @note This just set the current reference position of the motor, it does not change the actual position of the motor.
     *
     * @param position
     */
    virtual void setPosition(float position);

    /**
     * @brief Get the current speed of the motor in revolutions per minute (rpm)
     *
     * @return int16_t
     */
    virtual int16_t getRPM() const;

    /**
     * @brief Get the actual output current(or voltage) of the motor
     *
     * @return int16_t
     */
    virtual int16_t getActualCurrent() const;

    /**
     * @brief Get the temperature of the motor rotor
     *
     * @return uint8_t
     */
    uint8_t getRotorTemperature() const;

    /**
     * @brief Get the temperature of the motor MOS
     *
     * @return uint8_t
     */
    uint8_t getMosTemperature() const;
    /**
     * @brief Get the Reveice Count of the motor, this can be used to estimate the receive frequency of the motor
     *
     * @return uint32_t
     */

    virtual uint32_t getReveiceCount() const;

    /**
     * @brief Check if the motor is connected
     *
     * @return true
     * @return false
     */
    bool isConnected() const;

    /**
     * @brief Get the Error of the motor
     *
     * @return MotorERR
     */
    MotorERR getError() const;

    /**
     * @brief Get the Motor Mode of the motor
     *
     * @return MotorMode
     */
    MotorMode getMotorMode() const;

    /**
     * @brief Set the Motor Mode of the motor
     *
     * @param mode
     */
    void setMotorMode(uint8_t mode);
    void setID(uint8_t id);

    void setMIT(float positionSet, float rpmSet, float currentSet);
    void setTriLoop(float positionSet, float speedSet);
    void setSpeed(float speed);
    void clearErr();

    void sendMIT();
    void sendTriLoop();
    void sendSpeed();

    void send();

    MotorERR getError() { return motorError; }

    /**
     * @brief The array of all the possible DM4310s
     */
    static DM4310 dm4310Motors[4];

    /**
     * @attention   This function is used to decode the CAN message and update the motor data,
     *              you should not use this function.
     */

#ifdef HAL_CAN_MODULE_ENABLED
    static void decodeFeedback(CAN_HandleTypeDef *, uint32_t);
#elif defined HAL_FDCAN_MODULE_ENABLED
    static void decodeFeedback(FDCAN_HandleTypeDef *, uint32_t);
#endif

   protected:
    /**
     * @attention   You should not call this constructor directly.
     *              Instead, call DM4310::getMotor() to get the motor instance according to the motor CAN ID.
     */
    DM4310();

#ifdef HAL_CAN_MODULE_ENABLED
    friend void decodeFeedback(CAN_RxHeaderTypeDef &, uint8_t DM4310RxBuffer[8]);
#elif defined HAL_FDCAN_MODULE_ENABLED
    friend void decodeFeedback(FDCAN_RxHeaderTypeDef &, uint8_t DM4310RxBuffer[8]);
#endif

    volatile uint16_t rawEncoder;
    volatile uint16_t lastRawEncoder;
    volatile float position;
    volatile float rpm;
    volatile float rpmFeedback;
    volatile int16_t actualCurrent;
    volatile int16_t setCurrent;
    volatile uint16_t currentLimit;

    volatile uint8_t rotorTemperature;
    volatile uint8_t mosTemperature;
    volatile int32_t rotaryCnt;
    volatile float positionOffset;

    volatile uint32_t disconnectCnt;
    volatile uint32_t receiveCnt;
    volatile bool connected;
    volatile bool sendFlag = false;

    friend DM4310 &getMotor(uint8_t id);
    friend void motorUpdate(void *);
    friend void sendMotor(uint8_t motorID);

   private:
    static int32_t floatToUint(float x, float xMin, float xMax, int bits)
    {
        /// Converts a float to an unsigned int, given range and number of bits ///
        float span   = xMax - xMin;
        float offset = xMin;
        return (int32_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
    }

    static float uintToFloat(int xUint, float xMin, float xMax, int bits)
    {
        /// converts unsigned int to float, given range and number of bits ///
        float span   = xMax - xMin;
        float offset = xMin;
        return ((float)xUint) * span / ((float)((1 << bits) - 1)) + offset;
    }

    volatile uint8_t motorMode;
    volatile bool MotorEnabled;
    volatile MotorERR motorError;
    uint8_t motorID;
};

/**
 * @brief Get the DM4310 object according to the CAN ID
 *
 * @param canid (eg. 0x205)
 * @return DM4310&
 */
DM4310 &getMotor(uint32_t canid);

/**
 * @brief   Send the command to the motor by group,
 *          call this function after you set the output current(or voltage) of the motor.
 *
 * @param group     0 -> 0x200 , 1 -> 0x1ff, 2 -> 0x2ff
 */
void sendMotor(uint8_t motorID);

/**
 * @brief Initialize the DM4310 driver
 *          Call this function before using this DM4310 driver
 *
 * @note  If you do not want to use this DM4310 driver provided by us, do not call this function.
 */
void init();

#ifdef HAL_CAN_MODULE_ENABLED
void decodeFeedback(CAN_RxHeaderTypeDef &, uint8_t DM4310RxBuffer[8]);
#elif defined HAL_FDCAN_MODULE_ENABLED
void decodeFeedback(FDCAN_RxHeaderTypeDef &, uint8_t DM4310RxBuffer[8]);
#endif
}  // namespace DM4310
#endif