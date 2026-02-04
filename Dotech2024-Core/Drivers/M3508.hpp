/**
 * @file
 * @author jycjycjyc
 * @brief
 * @version
 * @date
 * @copyright
 */
#pragma once
#include "appConfig.hpp"

#ifndef USE_M3508
#define USE_M3508 0
#endif

#if USE_M3508 && defined(HAL_CAN_MODULE_ENABLED)

#include "main.h"

#ifdef HAL_CAN_MODULE_ENABLED
#include "can.h"
#ifndef M3508_CAN
#define M3508_CAN CAN_Handler
#endif
#elif defined HAL_FDCAN_MODULE_ENABLED
#define M3508_CAN hfdcan1
#endif
namespace M3508
{
class M3508
{
   public:
    M3508(const M3508 &)            = delete;
    M3508 &operator=(const M3508 &) = delete;

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
     * @brief Get the set output current(or voltage) of the motor
     *
     * @return int16_t
     */
    int16_t getOutputCurrent() const;

    /**
     * @brief Set the output current(or voltage) of the motor
     * @note  This function will limit the current(or voltage) according to the current(or voltage) limit of the motor.
     *        Please call sendMotorGroup() to send the command to the motor.
     *
     * @param current
     */
    virtual void setOutputCurrent(int32_t current);

    /**
     * @brief Set the Current(or voltage) Limit of the motor
     * @note  To avoid overflow,
     *          the maximum current limit for M3508 is 16384,
     *          and the maximum voltage limit for GM6020 is 30000.
     *
     * The default limit is 10000.
     *
     * @param current
     */
    void setCurrentLimit(uint16_t current);

    /**
     * @brief Get the temperature of the motor
     *
     * @return uint8_t
     */
    uint8_t getTemperature() const;

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
     * @brief The array of all the possible M3508s
     */
    static M3508 motors[11];

/**
 * @attention   This function is used to decode the CAN message and update the motor data,
 *              you should not use this function.
 */
#ifdef HAL_CAN_MODULE_ENABLED
    static void decodeFeedback(CAN_HandleTypeDef *, uint32_t rxFifoITs);
    friend void decodeFeedback(CAN_RxHeaderTypeDef &, uint8_t M3508RxBuffer[8]);
#elif defined HAL_FDCAN_MODULE_ENABLED
    static void decodeFeedback(FDCAN_HandleTypeDef *, uint32_t rxFifoITs);
    friend void decodeFeedback(FDCAN_RxHeaderTypeDef &, uint8_t M3508RxBuffer[8]);
#endif

   protected:
    /**
     * @attention   You should not call this constructor directly.
     *              Instead, call M3508::getMotor() to get the motor instance according to the motor CAN ID.
     */
    M3508();

    volatile uint16_t rawEncoder;
    volatile uint16_t lastRawEncoder;
    volatile float position;
    volatile int16_t rpm;
    volatile int16_t actualCurrent;
    volatile int16_t setCurrent;
    volatile uint16_t currentLimit;

    volatile uint8_t temperature;

    volatile int32_t rotaryCnt;
    volatile int16_t positionOffset;

    volatile uint32_t disconnectCnt;
    volatile uint32_t receiveCnt;
    volatile bool connected;

    friend M3508 &getMotor(uint8_t id);
    friend void motorUpdate(void *);
    friend void sendMotorGroup(uint32_t group);
};

/**
 * @brief Get the M3508 object according to the CAN ID
 *
 * @param canid (eg. 0x205)
 * @return M3508&
 */
M3508 &getMotor(uint32_t canid);

/**
 * @brief   Send the command to the motor by group,
 *          call this function after you set the output current(or voltage) of the motor.
 *
 * @param group     0 -> 0x200 , 1 -> 0x1ff, 2 -> 0x2ff
 */
void sendMotorGroup(uint32_t group);

/**
 * @brief Initialize the M3508 driver
 *          Call this function before using this M3508 driver
 *
 * @note  If you do not want to use this M3508 driver provided by us, do not call this function.
 */
void init();

#ifdef HAL_CAN_MODULE_ENABLED
void decodeFeedback(CAN_RxHeaderTypeDef &, uint8_t M3508RxBuffer[8]);
#elif defined HAL_FDCAN_MODULE_ENABLED
void decodeFeedback(FDCAN_RxHeaderTypeDef &, uint8_t M3508RxBuffer[8]);
#endif

}  // namespace M3508

#endif