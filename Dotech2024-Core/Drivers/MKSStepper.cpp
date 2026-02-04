#include "MKSStepper.hpp"

#if USE_MKSStepper && defined HAL_CAN_MODULE_ENABLED

#include <string.h>

#include "math.h"
#define CONSTRAIN(x, limit) ((x) > (limit) ? (limit) : ((x) < (-limit) ? (-limit) : (x)))
namespace Core
{
namespace Drivers
{
namespace MKSStepper
{

CAN_TxHeaderTypeDef canTxHeader;
// buffers
uint8_t txBuffer[8];
uint8_t rxBuffer[8];
SemaphoreHandle_t communcationSemaphore;
StaticSemaphore_t communcationSemaphoreBuffer;

int stepperCnt                       = 0;
MKSStepper *steppers[MKSStepper_NUM] = {nullptr};

uint32_t canMailBox;
void MKSStepper::appendCRC(uint8_t *data, uint8_t len)
{
    uint32_t crc = 0;
    for (int i = 0; i < len - 1; i++)
    {
        crc += data[i];
    }
    crc += this->canID;
    data[len - 1] = static_cast<uint8_t>(crc & 0xFF);
}

MKSStepper::MKSStepper(uint16_t canID_) : canID(canID_)
{
    configASSERT(stepperCnt < MKSStepper_NUM);
    steppers[stepperCnt] = this;
    stepperCnt++;
}
MKSStepper::Status MKSStepper::pollForCommuncation(uint16_t timeout)
{
    if (xSemaphoreTake(communcationSemaphore, timeout) == pdTRUE)
    {
        xSemaphoreGive(communcationSemaphore);
        if (this->comError)
            return Status::STATE_ERROR;
        else
            return Status::STATE_NORMAL;
    }
    else
    {
        xSemaphoreGive(communcationSemaphore);
        return Status::STATE_TIMEOUT;
    }
}
MKSStepper::Status MKSStepper::readEncoderCarry()
{
    this->request = Request::READ_ENCODER_CARRY;
    Status readStatus;
    if (xSemaphoreTake(communcationSemaphore, STEPPER_TIMEOUT) == false)

    {
        HAL_CAN_AbortTxRequest(&STEPPER_CAN, canMailBox);
    }
    canTxHeader.StdId = this->canID;
    canTxHeader.DLC   = DLC_ACCUM_ENCODER_CAN;
    txBuffer[0]       = CMD_CARRY_ENCODER;
    appendCRC(txBuffer, 2);
    if (HAL_CAN_GetTxMailboxesFreeLevel(&STEPPER_CAN) == 0)
        HAL_CAN_AbortTxRequest(&STEPPER_CAN, canMailBox);
    HAL_CAN_AddTxMessage(&STEPPER_CAN, &canTxHeader, txBuffer, &canMailBox);
    readStatus = pollForCommuncation(10);
    return readStatus;
}

MKSStepper::Status MKSStepper::readAccumulatedEncoder()
{
    this->request = Request::READ_ENCODER_ACCUMULATED;
    Status readStatus;
    if (xSemaphoreTake(communcationSemaphore, STEPPER_TIMEOUT) != pdTRUE)

    {
        HAL_CAN_AbortTxRequest(&STEPPER_CAN, canMailBox);
    }
    canTxHeader.StdId = this->canID;
    canTxHeader.DLC   = DLC_ACCUM_ENCODER_CAN;
    txBuffer[0]       = CMD_ACCUM_ENCODER;
    appendCRC(txBuffer, 2);
    if (HAL_CAN_GetTxMailboxesFreeLevel(&STEPPER_CAN) == 0)
        HAL_CAN_AbortTxRequest(&STEPPER_CAN, canMailBox);
    HAL_CAN_AddTxMessage(&STEPPER_CAN, &canTxHeader, txBuffer, &canMailBox);
    readStatus = pollForCommuncation(10);
    return readStatus;
}
MKSStepper::Status MKSStepper::readSpeed()
{
    this->request = Request::READ_SPEED;
    Status readStatus;
    if (xSemaphoreTake(communcationSemaphore, STEPPER_TIMEOUT) == false)
    {
        HAL_CAN_AbortTxRequest(&STEPPER_CAN, canMailBox);
    }
    canTxHeader.StdId = this->canID;
    canTxHeader.DLC   = DLC_READ_CMD;
    txBuffer[0]       = CMD_READ_SPEED;
    appendCRC(txBuffer, 2);
    if (HAL_CAN_GetTxMailboxesFreeLevel(&STEPPER_CAN) == 0)
        HAL_CAN_AbortTxRequest(&STEPPER_CAN, canMailBox);
    HAL_CAN_AddTxMessage(&STEPPER_CAN, &canTxHeader, txBuffer, &canMailBox);
    readStatus = pollForCommuncation(10);
    return readStatus;
}

MKSStepper::Status MKSStepper::readHome()
{
    this->request = Request::READ_HOME_STATE;
    Status readStatus;
    if (xSemaphoreTake(communcationSemaphore, STEPPER_TIMEOUT) == false)

    {
        HAL_CAN_AbortTxRequest(&STEPPER_CAN, canMailBox);
    }
    canTxHeader.StdId = this->canID;
    canTxHeader.DLC   = DLC_READ_CMD;
    txBuffer[0]       = CMD_BACK_HOME;
    appendCRC(txBuffer, 2);
    if (HAL_CAN_GetTxMailboxesFreeLevel(&STEPPER_CAN) == 0)
        HAL_CAN_AbortTxRequest(&STEPPER_CAN, canMailBox);
    HAL_CAN_AddTxMessage(&STEPPER_CAN, &canTxHeader, txBuffer, &canMailBox);
    readStatus = pollForCommuncation(10);
    return readStatus;
}

MKSStepper::Status MKSStepper::readPositionError(float &error)
{
    request = Request::READ_POSITION_ERROR;
    Status readStatus;
    if (xSemaphoreTake(communcationSemaphore, STEPPER_TIMEOUT) == false)

    {
        HAL_CAN_AbortTxRequest(&STEPPER_CAN, canMailBox);
    }
    canTxHeader.StdId = this->canID;
    canTxHeader.DLC   = DLC_READ_CMD;
    txBuffer[0]       = CMD_READ_ANGLE_ERROR;
    appendCRC(txBuffer, 2);
    if (HAL_CAN_GetTxMailboxesFreeLevel(&STEPPER_CAN) == 0)
        HAL_CAN_AbortTxRequest(&STEPPER_CAN, canMailBox);
    HAL_CAN_AddTxMessage(&STEPPER_CAN, &canTxHeader, txBuffer, &canMailBox);
    readStatus = pollForCommuncation(10);
    error      = this->feedback.encoderError / 65536.0f * 2 * static_cast<float>(M_PI);
    return readStatus;
}

MKSStepper::Status MKSStepper::readStall()
{
    request = Request::READ_STALL_STATE;
    Status readStatus;
    if (xSemaphoreTake(communcationSemaphore, STEPPER_TIMEOUT) == false)

    {
        HAL_CAN_AbortTxRequest(&STEPPER_CAN, canMailBox);
    }
    canTxHeader.StdId = this->canID;
    canTxHeader.DLC   = DLC_READ_CMD;
    txBuffer[0]       = CMD_READ_STALL;
    appendCRC(txBuffer, 2);
    if (HAL_CAN_GetTxMailboxesFreeLevel(&STEPPER_CAN) == 0)
        HAL_CAN_AbortTxRequest(&STEPPER_CAN, canMailBox);
    HAL_CAN_AddTxMessage(&STEPPER_CAN, &canTxHeader, txBuffer, &canMailBox);
    readStatus = pollForCommuncation(10);
    return readStatus;
}

MKSStepper::Status MKSStepper::clearStall()
{
    request = Request::CLEAR_STALL;
    Status readStatus;
    if (xSemaphoreTake(communcationSemaphore, STEPPER_TIMEOUT) == false)

    {
        HAL_CAN_AbortTxRequest(&STEPPER_CAN, canMailBox);
    }
    canTxHeader.StdId = this->canID;
    canTxHeader.DLC   = DLC_READ_CMD;
    txBuffer[0]       = CMD_CLEAR_STALL;
    appendCRC(txBuffer, 2);
    if (HAL_CAN_GetTxMailboxesFreeLevel(&STEPPER_CAN) == 0)
        HAL_CAN_AbortTxRequest(&STEPPER_CAN, canMailBox);
    HAL_CAN_AddTxMessage(&STEPPER_CAN, &canTxHeader, txBuffer, &canMailBox);
    readStatus = pollForCommuncation(10);
    return readStatus;
}

MKSStepper::Status MKSStepper::setSpeed(uint16_t speed, uint16_t acc, uint8_t dir)
{
    request = Request::SET_SPEED;
    Status readStatus;

    if (xSemaphoreTake(communcationSemaphore, STEPPER_TIMEOUT) != pdTRUE)

    {
        HAL_CAN_AbortTxRequest(&STEPPER_CAN, canMailBox);
    }
    canTxHeader.StdId = this->canID;
    canTxHeader.DLC   = 5;
    txBuffer[0]       = CMD_SET_SPEED;
    txBuffer[1]       = (dir << 7) | ((CONSTRAIN(speed, 3000) >> 8) & 0x0F);
    txBuffer[2]       = CONSTRAIN(speed, 255) & 0x00FF;
    txBuffer[3]       = CONSTRAIN(acc, 255);
    appendCRC(txBuffer, 5);
    if (HAL_CAN_GetTxMailboxesFreeLevel(&STEPPER_CAN) == 0)
        HAL_CAN_AbortTxRequest(&STEPPER_CAN, canMailBox);
    HAL_CAN_AddTxMessage(&STEPPER_CAN, &canTxHeader, txBuffer, &canMailBox);
    readStatus = pollForCommuncation(10);
    return readStatus;
}

MKSStepper::Status MKSStepper::setAbsAngle(uint16_t speed, uint8_t acc, float angle)
{
    int32_t absAxis = static_cast<int32_t>(angle / 2 / M_PI * 65536.0f);
    return setAbsPosition(3000, 255, absAxis);
}

MKSStepper::Status MKSStepper::goHome()
{
    request = Request::GO_HOME;
    Status readStatus;

    if (xSemaphoreTake(communcationSemaphore, STEPPER_TIMEOUT) != pdTRUE)

    {
        HAL_CAN_AbortTxRequest(&STEPPER_CAN, canMailBox);
    }
    canTxHeader.StdId = this->canID;
    canTxHeader.DLC   = 2;
    txBuffer[0]       = CMD_SET_HOME;

    appendCRC(txBuffer, 2);
    if (HAL_CAN_GetTxMailboxesFreeLevel(&STEPPER_CAN) == 0)
        HAL_CAN_AbortTxRequest(&STEPPER_CAN, canMailBox);
    HAL_CAN_AddTxMessage(&STEPPER_CAN, &canTxHeader, txBuffer, &canMailBox);
    readStatus = pollForCommuncation(10);
    return readStatus;
}

MKSStepper::Status MKSStepper::setAbsPosition(uint16_t speed, uint8_t acc, int32_t absAxis)
{
    request = Request::SET_ABS_POSITION;

    Status readStatus;

    if (xSemaphoreTake(communcationSemaphore, STEPPER_TIMEOUT) == false)
    {
        HAL_CAN_AbortTxRequest(&STEPPER_CAN, canMailBox);
    }
    canTxHeader.StdId = this->canID;
    canTxHeader.DLC   = 8;
    txBuffer[0]       = 0xF5;                    // 功能码
    txBuffer[1]       = (speed >> 8) & 0x00FF;   // 速度高8位
    txBuffer[2]       = speed & 0x00FF;          // 速度低8位
    txBuffer[3]       = acc;                     // 加速度
    txBuffer[4]       = (absAxis >> 16) & 0xFF;  // 绝对坐标 bit23 - bit16
    txBuffer[5]       = (absAxis >> 8) & 0xFF;   // 绝对坐标 bit15 - bit8
    txBuffer[6]       = (absAxis >> 0) & 0xFF;   // 绝对坐标 bit7 - bit0
    appendCRC(txBuffer, 8);

    if (HAL_CAN_GetTxMailboxesFreeLevel(&STEPPER_CAN) == 0)
        HAL_CAN_AbortTxRequest(&STEPPER_CAN, canMailBox);
    HAL_CAN_AddTxMessage(&STEPPER_CAN, &canTxHeader, txBuffer, &canMailBox);
    readStatus = pollForCommuncation(10);
    if(this->feedback.stall)
    {
        return Status::STATE_STALL;
    }
    // if (readStatus == Status::STATE_NORMAL)
    // {
    //     while (this->flags.positionReached == 1)
    //     {
    //         vTaskDelay(1);
    //     }
    // }

    return readStatus;
}

MKSStepper::Status MKSStepper::stopFromAbsPosProcess(uint8_t acc)
{
    request = Request::STOP;

    Status readStatus;

    if (xSemaphoreTake(communcationSemaphore, STEPPER_TIMEOUT) == false)
    {
        HAL_CAN_AbortTxRequest(&STEPPER_CAN, canMailBox);
    }
    canTxHeader.StdId = this->canID;
    canTxHeader.DLC   = 8;
    txBuffer[0]       = 0xF5;  // 功能码
    txBuffer[1]       = 0;     // 速度高8位
    txBuffer[2]       = 0;     // 速度低8位
    txBuffer[3]       = acc;   // 加速度
    txBuffer[4]       = 0;     // 绝对坐标 bit23 - bit16
    txBuffer[5]       = 0;     // 绝对坐标 bit15 - bit8
    txBuffer[6]       = 0;     // 绝对坐标 bit7 - bit0
    appendCRC(txBuffer, 8);

    if (HAL_CAN_GetTxMailboxesFreeLevel(&STEPPER_CAN) == 0)
        HAL_CAN_AbortTxRequest(&STEPPER_CAN, canMailBox);
    HAL_CAN_AddTxMessage(&STEPPER_CAN, &canTxHeader, txBuffer, &canMailBox);
    readStatus = pollForCommuncation(STEPPER_TIMEOUT);

    return readStatus;
}
void xAbsProcessFuncToPend(void *txBuf, uint32_t id)
{
    CAN_TxHeaderTypeDef header;
    header.IDE   = CAN_ID_STD;
    header.RTR   = CAN_RTR_DATA;
    header.ExtId = 0;
    header.StdId = id;
    header.DLC   = 8;
    if (HAL_CAN_GetTxMailboxesFreeLevel(&STEPPER_CAN) == 0)
        HAL_CAN_AbortTxRequest(&STEPPER_CAN, canMailBox);
    HAL_CAN_AddTxMessage(&STEPPER_CAN, &header, (uint8_t *)(txBuf), &canMailBox);
    if (xSemaphoreTake(communcationSemaphore, STEPPER_TIMEOUT) == pdTRUE)
    {
        ;  //
    }
    xSemaphoreGive(communcationSemaphore);
}
void MKSStepper::setAbsPositionFromISR(uint16_t speed, uint8_t acc, int32_t absAxis)
{
    request = Request::SET_ABS_POSITION;
    BaseType_t xHigherPriorityTaskWoken;

    if (xSemaphoreTakeFromISR(communcationSemaphore, NULL) == pdFALSE)
    {
        HAL_CAN_AbortTxRequest(&STEPPER_CAN, canMailBox);
    }
    txBuffer[0] = 0xF5;                    // 功能码
    txBuffer[1] = (speed >> 8) & 0x00FF;   // 速度高8位
    txBuffer[2] = speed & 0x00FF;          // 速度低8位
    txBuffer[3] = acc;                     // 加速度
    txBuffer[4] = (absAxis >> 16) & 0xFF;  // 绝对坐标 bit23 - bit16
    txBuffer[5] = (absAxis >> 8) & 0xFF;   // 绝对坐标 bit15 - bit8
    txBuffer[6] = (absAxis >> 0) & 0xFF;   // 绝对坐标 bit7 - bit0
    appendCRC(txBuffer, 8);
    xTimerPendFunctionCallFromISR(xAbsProcessFuncToPend, (void *)txBuffer, this->canID, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void MKSStepper::processHomeMsg()
{
    this->feedback.isHome = rxBuffer[1];
    this->flags.homeFlag  = this->feedback.isHome;
}

// nothing

void MKSStepper::processAccumulatedEncoder()
{
    this->feedback.accumulatedEncoder =
        /*(rxBuffer[1] << 40 | rxBuffer[2] << 32 |*/ (rxBuffer[3] << 24 | rxBuffer[4] << 16 | rxBuffer[5] << 8 | rxBuffer[6] << 0);
}

void MKSStepper::processEncoderCarryMsg()
{
    this->feedback.rawEncoder                  = (uint16_t)(rxBuffer[5] << 8 | rxBuffer[6] << 0);
    this->feedback.rawEncoderCarry             = (int32_t)(rxBuffer[1] << 24 | rxBuffer[2] << 16 | rxBuffer[3] << 8 | rxBuffer[4]);
    this->feedbackPosition.positionSingleRound = (this->feedback.rawEncoder) / 65536.0f * 2 * static_cast<float>(M_PI);
    this->feedbackPosition.positionAbs = this->feedbackPosition.positionSingleRound + this->feedback.rawEncoderCarry * 2 * static_cast<float>(M_PI);
}

void MKSStepper::processSetSpeed() { this->flags.speedFlag = rxBuffer[1]; }
void MKSStepper::processSpeedMsg()
{
    this->feedback.rpm = (int16_t)(rxBuffer[1] << 8 | rxBuffer[2] << 0);
    // noting
}
void MKSStepper::processPositionErrorMsg()
{
    this->feedback.encoderError          = (int16_t)(rxBuffer[1] << 8 | rxBuffer[2] << 0);
    this->feedbackPosition.positionError = this->feedback.encoderError / 65536.0f * 2 * static_cast<float>(M_PI);
    // nothing
}
void MKSStepper::processStallMsg()
{
    this->feedback.stall = rxBuffer[1];
    // nothing
}
void MKSStepper::processClearStall()
{
    this->feedback.stall  = rxBuffer[1];
    this->flags.stallFlag = rxBuffer[1];
    // nothing
}
void MKSStepper::processAbsPos()
{
    if (rxBuffer[1] == 0)
    {
        this->flags.positionReached = false;
        this->feedback.stall        = true;
    }
    else if (rxBuffer[1] == 2)
    {
        this->flags.positionReached = true;
    }
}
void MKSStepper::processBackHome()
{
    this->feedback.isHome = rxBuffer[1];
    this->flags.homeFlag  = this->feedback.isHome;
}
void MKSStepper::receiveCallbackImpl()
{
    request = Request::NONE;
    switch (rxBuffer[0])
    {
    case CMD_CARRY_ENCODER:
        processEncoderCarryMsg();
        break;
    case CMD_ACCUM_ENCODER:
        processAccumulatedEncoder();
        break;
    case CMD_READ_SPEED:
        processSpeedMsg();
        break;
    case CMD_READ_ANGLE_ERROR:
        processPositionErrorMsg();
        break;
    case CMD_BACK_HOME:
        processHomeMsg();
        break;
    case CMD_SET_ABS_POS:
        processAbsPos();
        break;
    case CMD_SET_SPEED:
        processSetSpeed();
        break;
    case CMD_SET_HOME:
        processBackHome();
        break;

    default:
        break;
    }
}

void receiveCallback(CAN_HandleTypeDef *hcan)
{
    //
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    static CAN_RxHeaderTypeDef canRxHeader;
    static uint8_t RxBuffer[8];
    while (HAL_CAN_GetRxFifoFillLevel(&STEPPER_CAN, CAN_RX_FIFO0))
    {
        HAL_CAN_GetRxMessage(&STEPPER_CAN, CAN_RX_FIFO0, &canRxHeader, (uint8_t *)&RxBuffer);
        for (int i = 0; i < MKSStepper_NUM; i++)
        {
            if (canRxHeader.StdId == steppers[i]->canID)
            {
                memcpy(rxBuffer, RxBuffer, sizeof(RxBuffer));
                steppers[i]->receiveCallbackImpl();
            }
        }
    }
    xSemaphoreGiveFromISR(communcationSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void init()
{
    communcationSemaphore = xSemaphoreCreateBinaryStatic(&communcationSemaphoreBuffer);
    configASSERT(communcationSemaphore != nullptr);
    // for (int i = 0; i < MKSStepper_NUM; i++)
    // {
    //     if (steppers[i] != nullptr)
    //     {
    //         steppers[i]->MKSStepper::rxAcknowledgeSemaphore = xSemaphoreCreateBinaryStatic(&steppers[i]->rxAcknowledgeSemaphoreBuffer);
    //     }
    // }
    CAN_FilterTypeDef stepperFilter = {
        0,                      // filterID HI
        0,                      // filterID LO
        0,                      // filterMask HI
        0,                      // filterMask LO
        CAN_FILTER_FIFO0,       // FIFO assignment
        0,                      // filter bank
        CAN_FILTERMODE_IDMASK,  // filter mode
        CAN_FILTERSCALE_16BIT,  // filter size
        CAN_FILTER_ENABLE,      // ENABLE or DISABLE
        0                       // Slave start bank
    };
    for (int i = 0; i < MKSStepper_NUM; i++)
    {
        canTxHeader.IDE   = CAN_ID_STD;
        canTxHeader.RTR   = CAN_RTR_DATA;
        canTxHeader.ExtId = 0;
    }
    configASSERT(HAL_CAN_ConfigFilter(&STEPPER_CAN, &stepperFilter) == HAL_OK);
    configASSERT(HAL_CAN_RegisterCallback(&STEPPER_CAN, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, receiveCallback) == HAL_OK);
    configASSERT(HAL_CAN_Start(&STEPPER_CAN) == HAL_OK);

    configASSERT(HAL_CAN_ActivateNotification(&STEPPER_CAN, CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK)
}
}  // namespace MKSStepper
}  // namespace Drivers
}  // namespace Core
#endif