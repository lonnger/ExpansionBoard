/**
 * @file MKSStepper.hpp
 * @author cooper_W cooper030517@gmail.com
 * @brief
 * @version 0.1
 * @date 2023-07-18
 *
 * @copyright Copyright (c) 2023
 *
 */
#pragma once
#include "appConfig.hpp"
#ifndef USE_MKSStepper
#define USE_MKSStepper 0
#endif

#if USE_MKSStepper && defined HAL_CAN_MODULE_ENABLED
#include "FreeRTOS.h"
#include "can.h"
#include "main.h"
#include "semphr.h"
#include "stdint.h"
#include "task.h"
#include "timers.h"

#ifndef MKSStepper_NUM
#define MKSStepper_NUM 20
#endif
/**
 * @brief Max current of MKSStepper stepper motor from 0 to 3000
 * @unit mA
 *
 */
#ifndef MKSStepper_MAX_CURRENT
#define MKSStepper_MAX_CURRENT 1600
#endif

/**
 * @brief which CAN bus to use
 *
 */
#define STEPPER_CAN hcan2

/**
 * @brief timeout for communication in polling mode
 *
 */
#define STEPPER_TIMEOUT 500

/**
 * @brief command code
 *
 */
#define DLC_ACCUM_ENCODER_CAN 2
#define DLC_READ_CMD 2
#define CMD_CARRY_ENCODER 0x30
#define CMD_ACCUM_ENCODER 0x31
#define CMD_READ_SPEED 0x32
#define CMD_READ_ANGLE_ERROR 0x39
#define CMD_BACK_HOME 0x3B
#define CMD_READ_STALL 0x3E
#define CMD_CLEAR_STALL 0x3D
#define CMD_SET_ABS_POS 0xF5
#define CMD_SET_SPEED 0xF6
#define CMD_SET_HOME 0x91

#if !defined(UNUSED)
#define UNUSED(x) (void)(x)
#endif
namespace Core
{
namespace Drivers
{
namespace MKSStepper
{
/**
 * @brief MKSStepper stepper motor driver
 *
 */
class MKSStepper
{
   public:
    /*********************constructor***********************/
    MKSStepper(uint16_t canID);
    MKSStepper()                         = delete;
    MKSStepper(const MKSStepper &)            = delete;
    MKSStepper &operator=(const MKSStepper &) = delete;

    /*********************Declaration***********************/
    /**
     * @brief feedback data, raw data from motor
     *
     */
    struct Feedback  // raw data
    {
        int32_t rawEncoderCarry;
        int32_t rawEncoder;
        int64_t accumulatedEncoder;
        int16_t rpm;
        int16_t encoderError;
        int32_t inputPulse;
        uint8_t stall;
        uint8_t isEnable;
        uint8_t isHome;
    };
    /**
     * @brief flag for motor, used for command feedback
     *
     */
    struct Flag
    {
        uint8_t homeFlag;
        uint8_t stallFlag;
        uint8_t positionReached;
        uint8_t speedFlag;
    };
    struct Angle  // processed data
    {
        float positionSingleRound;
        float positionAbs;
        float positionError;
    };
    /**
     * @brief Status Tag of the motor
     *
     */
    enum class Status
    {
        STATE_STALL,
        STATE_NORMAL,
        STATE_ERROR,
        STATE_TIMEOUT
    };
    /**
     * @brief current request Tag
     *
     */
    enum class Request
    {
        NONE,
        CMD_RET,
        READ_ENCODER_CARRY,
        READ_ENCODER_ACCUMULATED,
        READ_SPEED,
        READ_INPUT_PULSE,
        READ_CURRENT_POSITION,
        READ_POSITION_ERROR,
        READ_ENABLE_STATE,
        READ_STALL_STATE,
        READ_HOME_STATE,
        CLEAR_STALL,
        SET_SPEED,
        SET_ABS_POSITION,
        GO_HOME,
        STOP
    };
    /*********************static members***********************/

   public:
    /**
     * @brief wait for response from motor using semphore
     *
     */
    Status pollForCommuncation(uint16_t timeout);

    /**
     * @brief using carry mode to decode the encoder data
     */
    Status readEncoderCarry();
    void processEncoderCarryMsg();  // process callback function

    // accumulated encoder
    Status readAccumulatedEncoder();
    void processAccumulatedEncoder();  // process callback function
    //
    Status readSpeed();
    void processSpeedMsg();  // process callback function

    Status readPositionError(float &error);
    void processPositionErrorMsg();  // process callback function

    Status readHome();
    void processHomeMsg();  // process callback function

    Status goHome();
    void processBackHome();  // process callback function

    Status readStall();
    void processStallMsg();  // process callback function

    Status clearStall();
    void processClearStall();  // process callback function

    /**
     * @brief set postion by setting absolute angle
     * @param speed  the speed of stepper
     * @param acc    accelation of stepper
     * @param angle set abs angle
     * @warning use setAbsPositionFromISR in inerrupt
     */
    Status setAbsAngle(uint16_t speed, uint8_t acc, float angle);
    /**
     * @brief set postion by setting absolute axis
     */
    Status setAbsPosition(uint16_t speed, uint8_t acc, int32_t absAxis);
    Status stopFromAbsPosProcess(uint8_t acc = 0);

    void setAbsPositionFromISR(uint16_t speed, uint8_t acc, int32_t absAxis);
    void processAbsPos();

    Status setSpeed(uint16_t speed, uint16_t acc, uint8_t dir);
    void processSetSpeed();

    uint16_t canID;
    void receiveCallbackImpl();

    // SemaphoreHandle_t rxAcknowledgeSemaphore;
    // StaticSemaphore_t rxAcknowledgeSemaphoreBuffer;
    //    protected:
    bool comError          = false;
    Feedback feedback      = {};
    Flag flags             = {};
    Angle feedbackPosition = {};
    Status status          = Status::STATE_NORMAL;
    Request request        = Request::NONE;

    void appendCRC(uint8_t *data, uint8_t len);

   private:
    // void initImpl() { rxAcknowledgeSemaphore = xSemaphoreCreateBinaryStatic(&rxAcknowledgeSemaphoreBuffer); }

    // process function
    // read encoder
};

// CAN handle instance
extern CAN_TxHeaderTypeDef canTxHeader;
// buffers
extern uint8_t txBuffer[8];
extern uint8_t rxBuffer[8];
// communication sync semaphore
extern SemaphoreHandle_t communcationSemaphore;
extern StaticSemaphore_t communcationSemaphoreBuffer;

extern int stepperCnt;
extern MKSStepper *steppers[MKSStepper_NUM];
// init the module
void init();
void receiveCallback(CAN_HandleTypeDef *hcan);

}  // namespace MKSStepper
}  // namespace Drivers
}  // namespace Core
#endif
