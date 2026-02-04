#include "DM4310.hpp"

#if USE_DM4310 && defined(HAL_CAN_MODULE_ENABLED)

#include <cstdint>

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "cmsis_compiler.h"
#include "main.h"
#include "math.h"
#include "portable.h"
#include "semphr.h"
#include "task.h"

#define DM4310_TIMEOUT pdMS_TO_TICKS(200)

#include "can.h"

#ifndef DM4310_CAN
#define DM4310_CAN CAN_Handler
#endif

namespace DM4310
{
DM4310::DM4310()
    : rawEncoder(0),
      lastRawEncoder(0),
      position(0),
      rpm(0),
      actualCurrent(0),
      currentLimit(10000),
      rotorTemperature(0),
      mosTemperature(0),
      rotaryCnt(0),
      positionOffset(0),
      disconnectCnt(200),
      receiveCnt(0),
      connected(false),
      motorMode(1),
      MotorEnabled(0),
      motorID(0)
{
}

DM4310 DM4310::dm4310Motors[4];

uint16_t DM4310::getRawEncoder() const { return this->rawEncoder; }

float DM4310::getPosition() const { return this->position; }

void DM4310::setID(uint8_t id) { this->motorID = id; }

void DM4310::setPosition(float setPosition)
{
    uint32_t ulOriginalBASEPRI = 0;
    if (__get_IPSR())
        ulOriginalBASEPRI = taskENTER_CRITICAL_FROM_ISR();
    else
        taskENTER_CRITICAL();

    this->positionOffset = (float)((setPosition - this->position));

    if (__get_IPSR())
        taskEXIT_CRITICAL_FROM_ISR(ulOriginalBASEPRI);
    else
        taskEXIT_CRITICAL();
}

int16_t DM4310::getRPM() const { return this->rpm; }

int16_t DM4310::getActualCurrent() const { return this->actualCurrent; }

uint8_t DM4310::getRotorTemperature() const { return this->rotorTemperature; }

uint8_t DM4310::getMosTemperature() const { return this->mosTemperature; }

uint32_t DM4310::getReveiceCount() const { return this->receiveCnt; }

bool DM4310::isConnected() const { return this->connected; }

DM4310 &getMotor(uint32_t id)
{
    // canid -= STD_ID;
    configASSERT(id < 4);
    return DM4310::dm4310Motors[id];
}

void DM4310::setMotorMode(uint8_t mode) { this->motorMode = mode; }

void DM4310::enableMotor()
{
    volatile uint8_t canTxEnableData[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    CAN_TxHeaderTypeDef canEnableTxHeader = {0x201 + motorID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, DISABLE};

    static uint32_t canMailBox[3];
    static volatile uint32_t canLastSend = 0;
    static volatile uint32_t canFree1;
    canFree1 = HAL_CAN_GetTxMailboxesFreeLevel(&DM4310_CAN);
    if (HAL_CAN_GetTxMailboxesFreeLevel(&DM4310_CAN) == 0)
    {
        canMailBox[0] = HAL_CAN_GetTxMailboxesFreeLevel(&DM4310_CAN);
        HAL_CAN_AbortTxRequest(&DM4310_CAN, canMailBox[0]);
    }
    HAL_CAN_AddTxMessage(&DM4310_CAN, &canEnableTxHeader, (uint8_t *)canTxEnableData, &canMailBox[0]);
}

void DM4310::clearErr()
{
    static volatile uint8_t canTxClearErrData[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};
    CAN_TxHeaderTypeDef canTxHeader = {0x00 + 0x201, 0, CAN_ID_STD, CAN_RTR_DATA, 8, DISABLE};
    canTxHeader.StdId          = 0x00 + 0x201 + motorID;
    static uint32_t canMailBox[3];
    HAL_CAN_AddTxMessage(&DM4310_CAN, &canTxHeader, (uint8_t *)canTxClearErrData, &canMailBox[0]);
}

void DM4310::disableMotor()
{
    volatile uint8_t canTxData[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    CAN_TxHeaderTypeDef canTxHeader = {0x00 + 0x201 + motorID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, DISABLE};
    static uint32_t canMailBox[3];
    HAL_CAN_AddTxMessage(&DM4310_CAN, &canTxHeader, (uint8_t *)canTxData, &canMailBox[0]);
}

void DM4310::setMIT(float positionSet, float rpmSet, float currentSet) { configASSERT(false); }
void DM4310::setTriLoop(float positionSet, float rpmSet)
{
    setMotorMode(1);
    this->position = positionSet;
    this->rpm      = rpmSet;
}

void DM4310::setSpeed(float speed)
{
    setMotorMode(2);
    this->rpm = speed;
}
void DM4310::sendMIT() {}
void DM4310::sendTriLoop()
{
    int8_t canTxData[8];
    CAN_TxHeaderTypeDef canTxHeader = {0x01 + 0x100 + motorID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, DISABLE};
    canTxData[0]           = *((uint8_t *)&this->position);
    canTxData[1]           = *((uint8_t *)&this->position + 1);
    canTxData[2]           = *((uint8_t *)&this->position + 2);
    canTxData[3]           = *((uint8_t *)&this->position + 3);
    canTxData[4]           = *((uint8_t *)&this->rpm);
    canTxData[5]           = *((uint8_t *)&this->rpm + 1);
    canTxData[6]           = *((uint8_t *)&this->rpm + 2);
    canTxData[7]           = *((uint8_t *)&this->rpm + 3);
    static uint32_t canMailBox[3];
    HAL_CAN_AddTxMessage(&DM4310_CAN, &canTxHeader, (uint8_t *)canTxData, &canMailBox[0]);
}

void DM4310:: sendSpeed()
{
    static int8_t canTxData[8];
    CAN_TxHeaderTypeDef canTxHeader = {0x000U + 0x201U + motorID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, DISABLE};
    canTxData[0]                    = *((uint8_t *)&this->rpm);
    canTxData[1]                    = *((uint8_t *)&this->rpm + 1);
    canTxData[2]                    = *((uint8_t *)&this->rpm + 2);
    canTxData[3]                    = *((uint8_t *)&this->rpm + 3);
    canTxData[4]                    = 0;
    canTxData[5]                    = 0;
    canTxData[6]                    = 0;
    canTxData[7]                    = 0;
    static uint32_t canMailBox[3];
    // static volatile uint32_t canLastSend = 0;
    // static volatile uint32_t canFree;

    // if (HAL_FDCAN_GetTxFifoFreeLevel(&DM4310_CAN) == 0)
    // {
    //     canMailBox[0] = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&DM4310_CAN);
    //     HAL_FDCAN_AbortTxRequest(&DM4310_CAN, canMailBox[0]);
    // }
    // HAL_FDCAN_AddMessageToTxFifoQ(&DM4310_CAN, &canTxHeader, (uint8_t *)canTxData);
    if (HAL_CAN_GetTxMailboxesFreeLevel(&DM4310_CAN) == 0)
    {
        canMailBox[0] = HAL_CAN_GetTxMailboxesFreeLevel(&DM4310_CAN);
        HAL_CAN_AbortTxRequest(&DM4310_CAN, canMailBox[0]);
    }
    HAL_CAN_AddTxMessage(&DM4310_CAN, &canTxHeader, (uint8_t *)canTxData, &canMailBox[0]);
}
void DM4310::send()
{
    switch (this->motorMode)
    {
    case 0:
        break;
    case 1:
        sendTriLoop();
        break;
    case 2:
        sendSpeed();
        break;
    default:
        break;
    }
}
void sendMotor(uint8_t motorID)
{
    // uint32_t ulOriginalBASEPRI = 0;
    // if (__get_IPSR())
    //     ulOriginalBASEPRI = taskENTER_CRITICAL_FROM_ISR();
    // else
    //     taskENTER_CRITICAL();
    DM4310::dm4310Motors[motorID].sendFlag = true;
    DM4310::dm4310Motors[motorID].send();
    // if (__get_IPSR())
    //     taskEXIT_CRITICAL_FROM_ISR(ulOriginalBASEPRI);
    // else
    //     taskEXIT_CRITICAL();
}

static StackType_t uxMotorUpdateTaskStack[128];
static StaticTask_t xMotorUpdateTaskTCB;
xTaskHandle xMotorSendTaskHandle;
xTaskHandle xMotorUpdateTaskHandle;
StaticSemaphore_t motorUpdateMutexBuffer;
SemaphoreHandle_t motorUpdateMutex;
void motorUpdate(void *)
{
    // static TickType_t xLastWakeTime = xTaskGetTickCount();
    // static uint32_t taskcnt         = 0;
    // while (true)
    // {
    // taskcnt++;
    // vTaskDelay(1000);
    for (uint32_t motorNum = 0; motorNum < 4; motorNum++)
    {
        if (DM4310::dm4310Motors[motorNum].sendFlag == true && DM4310::dm4310Motors[motorNum].disconnectCnt++ > DM4310_TIMEOUT)
        {
            DM4310::dm4310Motors[motorNum].sendFlag      = false;
            DM4310::dm4310Motors[motorNum].connected     = false;
            DM4310::dm4310Motors[motorNum].disconnectCnt = DM4310_TIMEOUT;
        }
    }
    // vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(2));
    // }
}

void DM4310::decodeFeedback(CAN_HandleTypeDef *, uint32_t)
{
    while (HAL_CAN_GetRxFifoFillLevel(&DM4310_CAN, CAN_RX_FIFO0))
    {
        uint32_t ulOriginalBASEPRI = taskENTER_CRITICAL_FROM_ISR();
        CAN_RxHeaderTypeDef DM4310canRxHeader;
        DM4310canRxHeader.StdId = 0;
        static uint8_t DM4310RxBuffer[8];
        int32_t position;
        int32_t rpm;
        int32_t current;
        HAL_CAN_GetRxMessage(&DM4310_CAN, CAN_RX_FIFO0, &DM4310canRxHeader, (uint8_t *)&DM4310RxBuffer);
        uint8_t motorID = DM4310RxBuffer[0] & 0x0F;
        motorID %= 0x100;
        motorID -= 1;
        DM4310::dm4310Motors[motorID].connected        = true;
        DM4310::dm4310Motors[motorID].disconnectCnt    = 0;
        DM4310::dm4310Motors[motorID].motorError       = (MotorERR)(DM4310RxBuffer[0] >> 4);
        position                                       = DM4310RxBuffer[1] << 8 | DM4310RxBuffer[2];
        rpm                                            = (DM4310RxBuffer[3] << 4) | (DM4310RxBuffer[4] >> 4);
        current                                        = (DM4310RxBuffer[4] & 0x0F) << 8 | DM4310RxBuffer[5];
        DM4310::dm4310Motors[motorID].mosTemperature   = DM4310RxBuffer[6];
        DM4310::dm4310Motors[motorID].rotorTemperature = DM4310RxBuffer[7];
        DM4310::dm4310Motors[motorID].position         = DM4310::uintToFloat(position, -12.5, 12.5, 16);

        DM4310::dm4310Motors[motorID].rpmFeedback   = DM4310::uintToFloat(rpm, -30, 30, 12);
        DM4310::dm4310Motors[motorID].actualCurrent = DM4310::uintToFloat(current, -10, 10, 12);
        DM4310::dm4310Motors[motorID].disconnectCnt = 0;
        DM4310::dm4310Motors[motorID].connected     = true;
        taskEXIT_CRITICAL_FROM_ISR(ulOriginalBASEPRI);
    }
}
void decodeFeedback(CAN_RxHeaderTypeDef &, uint8_t DM4310RxBuffer[8])
{
    int32_t position;
    int32_t rpm;
    int32_t current;
    uint8_t motorID = DM4310RxBuffer[0] & 0x0F;
    motorID %= 0x100;
    motorID -= 1;
    DM4310::dm4310Motors[motorID].connected        = true;
    DM4310::dm4310Motors[motorID].disconnectCnt    = 0;
    DM4310::dm4310Motors[motorID].motorError       = (MotorERR)(DM4310RxBuffer[0] >> 4);
    position                                       = DM4310RxBuffer[1] << 8 | DM4310RxBuffer[2];
    rpm                                            = (DM4310RxBuffer[3] << 4) | (DM4310RxBuffer[4] >> 4);
    current                                        = (DM4310RxBuffer[4] & 0x0F) << 8 | DM4310RxBuffer[5];
    DM4310::dm4310Motors[motorID].mosTemperature   = DM4310RxBuffer[6];
    DM4310::dm4310Motors[motorID].rotorTemperature = DM4310RxBuffer[7];
    DM4310::dm4310Motors[motorID].position         = DM4310::uintToFloat(position, -12.5, 12.5, 16);

    DM4310::dm4310Motors[motorID].rpmFeedback   = DM4310::uintToFloat(rpm, -30, 30, 12);
    DM4310::dm4310Motors[motorID].actualCurrent = DM4310::uintToFloat(current, -10, 10, 12);
    // uint32_t ulOriginalBASEPRI                  = taskENTER_CRITICAL_FROM_ISR();
    DM4310::dm4310Motors[motorID].disconnectCnt = 0;
    DM4310::dm4310Motors[motorID].connected     = true;
    // taskEXIT_CRITICAL_FROM_ISR(ulOriginalBASEPRI);
}
static StackType_t uxdm4310MotorsendTaskStack[128];
static StaticTask_t xdm4310MotorsendTaskTCB;
void sendMotorTask(void *params)
{
    // DM4310::dm4310Motors[0].disableMotor();
    // vTaskDelay(30);
    // DM4310::dm4310Motors[0].enableMotor();
    // DM4310::dm4310Motors[1].enableMotor();

    while (true)
    {
        if (DM4310::dm4310Motors[0].isConnected())
        {
            sendMotor(0);
        }
        else
        {
            DM4310::dm4310Motors[0].enableMotor();
        }

        vTaskDelay(1);

        if (DM4310::dm4310Motors[1].isConnected())
        {
            sendMotor(1);
        }
        else
        {
            DM4310::dm4310Motors[1].enableMotor();
        }
        // sendMotor(0);
        // sendMotor(1);
        // DM4310::dm4310Motors[1].enableMotor();
        // DM4310::dm4310Motors[0].enableMotor();

        // for (int i = 0; i < 2; i++)
        //     if (DM4310::dm4310Motors[i].getError() == MotorERR::LOSE_CONNECTION)
        //     {
        //         DM4310::dm4310Motors[i].clearErr();
        //         vTaskDelay(1);
        //         DM4310::dm4310Motors[i].enableMotor();
        //     }
        // xSemaphoreGive(motorUpdateMutex);  // schedule sync with motor update task
        motorUpdate(NULL);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void init()
{
    // FDCAN_FilterTypeDef DM4310Filter = {
    //     0,                      // filterID HI
    //     0,                      // filterID LO
    //     0,                      // filterMask HI
    //     0,                      // filterMask LO
    //     0,                      // FIFO assignment
    //     FDCAN_FILTER_TO_RXFIFO0,       // filterBank number
    //     CAN_FILTERMODE_IDMASK,  // filter mode
    //     CAN_FILTERSCALE_16BIT,  // filter size
    //     CAN_FILTER_ENABLE,      // ENABLE or DISABLE
    //     0                       // Slave start bank
    // };
    // configASSERT(HAL_FDCAN_ActivateNotification(&DM4310_CAN, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, (uint32_t)NULL) == HAL_OK);
    // configASSERT(HAL_FDCAN_RegisterRxFifo0Callback(&DM4310_CAN, DM4310::decodeFeedback) == HAL_OK);
    // configASSERT(HAL_FDCAN_Start(&DM4310_CAN) == HAL_OK);
    // motorUpdateMutex = xSemaphoreCreateBinaryStatic(&motorUpdateMutexBuffer);
    // xSemaphoreGive(motorUpdateMutex);
    DM4310::dm4310Motors[0].setID(0);
    DM4310::dm4310Motors[1].setID(1);
    DM4310::dm4310Motors[2].setID(2);
    // xTaskCreateStatic(motorUpdate, "MotorUpdate", 128, NULL, 10, uxMotorUpdateTaskStack, &xMotorUpdateTaskTCB);
    xTaskCreateStatic(sendMotorTask, "dm4310Motorsend", 128, NULL, 10, uxdm4310MotorsendTaskStack, &xdm4310MotorsendTaskTCB);
}

}  // namespace DM4310

#elif defined HAL_FDCAN_MODULE_ENABLED

#include "fdcan.h"

#define DM4310_CAN hfdcan1

namespace DM4310
{
DM4310::DM4310()
    : rawEncoder(0),
      lastRawEncoder(0),
      position(0),
      rpm(0),
      actualCurrent(0),
      currentLimit(10000),
      rotorTemperature(0),
      mosTemperature(0),
      rotaryCnt(0),
      positionOffset(0),
      disconnectCnt(200),
      receiveCnt(0),
      connected(false),
      motorMode(1),
      MotorEnabled(0),
      motorID(0)
{
}

DM4310 DM4310::dm4310Motors[4];

uint16_t DM4310::getRawEncoder() const { return this->rawEncoder; }

float DM4310::getPosition() const { return this->position; }

void DM4310::setID(uint8_t id) { this->motorID = id; }

void DM4310::setPosition(float setPosition)
{
    uint32_t ulOriginalBASEPRI = 0;
    if (__get_IPSR())
        ulOriginalBASEPRI = taskENTER_CRITICAL_FROM_ISR();
    else
        taskENTER_CRITICAL();

    this->positionOffset = (float)((setPosition - this->position));

    if (__get_IPSR())
        taskEXIT_CRITICAL_FROM_ISR(ulOriginalBASEPRI);
    else
        taskEXIT_CRITICAL();
}

int16_t DM4310::getRPM() const { return this->rpm; }

int16_t DM4310::getActualCurrent() const { return this->actualCurrent; }

uint8_t DM4310::getRotorTemperature() const { return this->rotorTemperature; }

uint8_t DM4310::getMosTemperature() const { return this->mosTemperature; }

uint32_t DM4310::getReveiceCount() const { return this->receiveCnt; }

bool DM4310::isConnected() const { return this->connected; }

DM4310 &getMotor(uint32_t id)
{
    // canid -= STD_ID;
    configASSERT(id < 4);
    return DM4310::dm4310Motors[id];
}

void DM4310::setMotorMode(uint8_t mode) { this->motorMode = mode; }

void DM4310::enableMotor()
{
    volatile uint8_t canTxEnableData[8]       = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    FDCAN_TxHeaderTypeDef fdcanEnableTxHeader = {0x00 + 0x201,
                                                 FDCAN_STANDARD_ID,
                                                 FDCAN_DATA_FRAME,
                                                 FDCAN_DLC_BYTES_8,
                                                 FDCAN_ESI_ACTIVE,
                                                 FDCAN_BRS_OFF,
                                                 FDCAN_CLASSIC_CAN,
                                                 FDCAN_NO_TX_EVENTS,
                                                 0};
    fdcanEnableTxHeader.Identifier            = 0x00 + 0x201 + motorID;
    static volatile uint32_t canMailBox[3];
    static volatile uint32_t canLastSend = 0;
    static volatile uint32_t canFree1;
    canFree1 = HAL_FDCAN_GetTxFifoFreeLevel(&DM4310_CAN);
    if (HAL_FDCAN_GetTxFifoFreeLevel(&DM4310_CAN) == 0)
    {
        canMailBox[0] = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&DM4310_CAN);
        HAL_FDCAN_AbortTxRequest(&DM4310_CAN, canMailBox[0]);
    }
    HAL_FDCAN_AddMessageToTxFifoQ(&DM4310_CAN, &fdcanEnableTxHeader, (uint8_t *)canTxEnableData);
}

void DM4310::clearErr()
{
    static volatile uint8_t canTxClearErrData[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};
    static FDCAN_TxHeaderTypeDef fdcanTxHeader   = {0x00 + 0x201,
                                                    FDCAN_STANDARD_ID,
                                                    FDCAN_DATA_FRAME,
                                                    FDCAN_DLC_BYTES_8,
                                                    FDCAN_ESI_ACTIVE,
                                                    FDCAN_BRS_OFF,
                                                    FDCAN_CLASSIC_CAN,
                                                    FDCAN_NO_TX_EVENTS,
                                                    0};
    fdcanTxHeader.Identifier                     = 0x00 + 0x201 + motorID;
    HAL_FDCAN_AddMessageToTxFifoQ(&DM4310_CAN, &fdcanTxHeader, (uint8_t *)canTxClearErrData);
}

void DM4310::disableMotor()
{
    volatile uint8_t canTxData[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};

    static FDCAN_TxHeaderTypeDef canTxHeader = {STD_ID + motorID,
                                                FDCAN_STANDARD_ID,
                                                FDCAN_DATA_FRAME,
                                                FDCAN_DLC_BYTES_8,
                                                FDCAN_ESI_ACTIVE,
                                                FDCAN_BRS_OFF,
                                                FDCAN_CLASSIC_CAN,
                                                FDCAN_NO_TX_EVENTS,
                                                0};
    canTxHeader.Identifier                   = 0x00 + 0x201 + motorID;
    HAL_FDCAN_AddMessageToTxFifoQ(&DM4310_CAN, &canTxHeader, (uint8_t *)canTxData);
}

void DM4310::setMIT(float positionSet, float rpmSet, float currentSet) { configASSERT(false); }
void DM4310::setTriLoop(float positionSet, float rpmSet)
{
    setMotorMode(1);
    this->position = positionSet;
    this->rpm      = rpmSet;
}

void DM4310::setSpeed(float speed)
{
    setMotorMode(2);
    this->rpm = speed;
}
void DM4310::sendMIT() {}
void DM4310::sendTriLoop()
{
    int8_t canTxData[8];
    // static uint32_t canMailBox;
    static FDCAN_TxHeaderTypeDef canTxHeader = {STD_ID + motorID,
                                                FDCAN_STANDARD_ID,
                                                FDCAN_DATA_FRAME,
                                                FDCAN_DLC_BYTES_8,
                                                FDCAN_ESI_ACTIVE,
                                                FDCAN_BRS_OFF,
                                                FDCAN_CLASSIC_CAN,
                                                FDCAN_NO_TX_EVENTS,
                                                0};
    canTxHeader.Identifier                   = 0x01 + 0x100 + motorID;
    canTxData[0]                             = *((uint8_t *)&this->position);
    canTxData[1]                             = *((uint8_t *)&this->position + 1);
    canTxData[2]                             = *((uint8_t *)&this->position + 2);
    canTxData[3]                             = *((uint8_t *)&this->position + 3);
    canTxData[4]                             = *((uint8_t *)&this->rpm);
    canTxData[5]                             = *((uint8_t *)&this->rpm + 1);
    canTxData[6]                             = *((uint8_t *)&this->rpm + 2);
    canTxData[7]                             = *((uint8_t *)&this->rpm + 3);
    HAL_FDCAN_AddMessageToTxFifoQ(&DM4310_CAN, &canTxHeader, (uint8_t *)canTxData);
}

void DM4310::sendSpeed()
{
    static int8_t canTxData[8];
    // static uint32_t canMailBox;
    FDCAN_TxHeaderTypeDef canTxHeader = {STD_ID + motorID,
                                         FDCAN_STANDARD_ID,
                                         FDCAN_DATA_FRAME,
                                         FDCAN_DLC_BYTES_8,
                                         FDCAN_ESI_ACTIVE,
                                         FDCAN_BRS_OFF,
                                         FDCAN_CLASSIC_CAN,
                                         FDCAN_NO_TX_EVENTS,
                                         0};
    canTxHeader.Identifier            = 0x00 + 0x201 + motorID;
    canTxData[0]                      = *((uint8_t *)&this->rpm);
    canTxData[1]                      = *((uint8_t *)&this->rpm + 1);
    canTxData[2]                      = *((uint8_t *)&this->rpm + 2);
    canTxData[3]                      = *((uint8_t *)&this->rpm + 3);
    canTxData[4]                      = 0;
    canTxData[5]                      = 0;
    canTxData[6]                      = 0;
    canTxData[7]                      = 0;
    static volatile uint32_t canMailBox[3];
    static volatile uint32_t canLastSend = 0;
    static volatile uint32_t canFree;

    if (HAL_FDCAN_GetTxFifoFreeLevel(&DM4310_CAN) == 0)
    {
        canMailBox[0] = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&DM4310_CAN);
        HAL_FDCAN_AbortTxRequest(&DM4310_CAN, canMailBox[0]);
    }
    HAL_FDCAN_AddMessageToTxFifoQ(&DM4310_CAN, &canTxHeader, (uint8_t *)canTxData);
}
void DM4310::send()
{
    switch (this->motorMode)
    {
    case 0:
        break;
    case 1:
        sendTriLoop();
        break;
    case 2:
        sendSpeed();
        break;
    default:
        break;
    }
}
void sendMotor(uint8_t motorID)
{
    // uint32_t ulOriginalBASEPRI = 0;
    // if (__get_IPSR())
    //     ulOriginalBASEPRI = taskENTER_CRITICAL_FROM_ISR();
    // else
    //     taskENTER_CRITICAL();
    DM4310::dm4310Motors[motorID].sendFlag = true;
    DM4310::dm4310Motors[motorID].send();
    // if (__get_IPSR())
    //     taskEXIT_CRITICAL_FROM_ISR(ulOriginalBASEPRI);
    // else
    //     taskEXIT_CRITICAL();
}

static StackType_t uxMotorUpdateTaskStack[128];
static StaticTask_t xMotorUpdateTaskTCB;
xTaskHandle xMotorSendTaskHandle;
xTaskHandle xMotorUpdateTaskHandle;
StaticSemaphore_t motorUpdateMutexBuffer;
SemaphoreHandle_t motorUpdateMutex;
void motorUpdate(void *)
{
    // static TickType_t xLastWakeTime = xTaskGetTickCount();
    // static uint32_t taskcnt         = 0;
    // while (true)
    // {
    // taskcnt++;
    vTaskDelay(1000);
    for (uint32_t motorNum = 0; motorNum < 4; motorNum++)
    {
        if (DM4310::dm4310Motors[motorNum].sendFlag == true && DM4310::dm4310Motors[motorNum].disconnectCnt++ > DM4310_TIMEOUT)
        {
            DM4310::dm4310Motors[motorNum].sendFlag      = false;
            DM4310::dm4310Motors[motorNum].connected     = false;
            DM4310::dm4310Motors[motorNum].disconnectCnt = DM4310_TIMEOUT;
        }
    }
    // vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(2));
    // }
}

void DM4310::decodeFeedback(FDCAN_HandleTypeDef *, uint32_t)
{
    while (HAL_FDCAN_GetRxFifoFillLevel(&DM4310_CAN, FDCAN_RX_FIFO0))
    {
        uint32_t ulOriginalBASEPRI = taskENTER_CRITICAL_FROM_ISR();
        FDCAN_RxHeaderTypeDef DM4310canRxHeader;
        DM4310canRxHeader.Identifier = 0;
        static uint8_t DM4310RxBuffer[8];
        int32_t position;
        int32_t rpm;
        int32_t current;
        HAL_FDCAN_GetRxMessage(&DM4310_CAN, FDCAN_RX_FIFO0, &DM4310canRxHeader, (uint8_t *)&DM4310RxBuffer);
        uint8_t motorID = DM4310RxBuffer[0] & 0x0F;
        motorID %= 0x100;
        motorID -= 1;
        DM4310::dm4310Motors[motorID].connected        = true;
        DM4310::dm4310Motors[motorID].disconnectCnt    = 0;
        DM4310::dm4310Motors[motorID].motorError       = (MotorERR)(DM4310RxBuffer[0] >> 4);
        position                                       = DM4310RxBuffer[1] << 8 | DM4310RxBuffer[2];
        rpm                                            = (DM4310RxBuffer[3] << 4) | (DM4310RxBuffer[4] >> 4);
        current                                        = (DM4310RxBuffer[4] & 0x0F) << 8 | DM4310RxBuffer[5];
        DM4310::dm4310Motors[motorID].mosTemperature   = DM4310RxBuffer[6];
        DM4310::dm4310Motors[motorID].rotorTemperature = DM4310RxBuffer[7];
        DM4310::dm4310Motors[motorID].position         = DM4310::uintToFloat(position, -12.5, 12.5, 16);

        DM4310::dm4310Motors[motorID].rpmFeedback   = DM4310::uintToFloat(rpm, -30, 30, 12);
        DM4310::dm4310Motors[motorID].actualCurrent = DM4310::uintToFloat(current, -10, 10, 12);
        DM4310::dm4310Motors[motorID].disconnectCnt = 0;
        DM4310::dm4310Motors[motorID].connected     = true;
        taskEXIT_CRITICAL_FROM_ISR(ulOriginalBASEPRI);
    }
}
void decodeFeedback(FDCAN_RxHeaderTypeDef &, uint8_t DM4310RxBuffer[8])
{
    int32_t position;
    int32_t rpm;
    int32_t current;
    uint8_t motorID = DM4310RxBuffer[0] & 0x0F;
    motorID %= 0x100;
    motorID -= 1;
    DM4310::dm4310Motors[motorID].connected        = true;
    DM4310::dm4310Motors[motorID].disconnectCnt    = 0;
    DM4310::dm4310Motors[motorID].motorError       = (MotorERR)(DM4310RxBuffer[0] >> 4);
    position                                       = DM4310RxBuffer[1] << 8 | DM4310RxBuffer[2];
    rpm                                            = (DM4310RxBuffer[3] << 4) | (DM4310RxBuffer[4] >> 4);
    current                                        = (DM4310RxBuffer[4] & 0x0F) << 8 | DM4310RxBuffer[5];
    DM4310::dm4310Motors[motorID].mosTemperature   = DM4310RxBuffer[6];
    DM4310::dm4310Motors[motorID].rotorTemperature = DM4310RxBuffer[7];
    DM4310::dm4310Motors[motorID].position         = DM4310::uintToFloat(position, -12.5, 12.5, 16);

    DM4310::dm4310Motors[motorID].rpmFeedback   = DM4310::uintToFloat(rpm, -30, 30, 12);
    DM4310::dm4310Motors[motorID].actualCurrent = DM4310::uintToFloat(current, -10, 10, 12);
    // uint32_t ulOriginalBASEPRI                  = taskENTER_CRITICAL_FROM_ISR();
    DM4310::dm4310Motors[motorID].disconnectCnt = 0;
    DM4310::dm4310Motors[motorID].connected     = true;
    // taskEXIT_CRITICAL_FROM_ISR(ulOriginalBASEPRI);
}
static StackType_t uxdm4310MotorsendTaskStack[128];
static StaticTask_t xdm4310MotorsendTaskTCB;
void sendMotorTask(void *params)
{
    // DM4310::dm4310Motors[0].disableMotor();
    // vTaskDelay(30);
    // DM4310::dm4310Motors[0].enableMotor();
    // DM4310::dm4310Motors[1].enableMotor();

    while (true)
    {
        if (DM4310::dm4310Motors[0].isConnected())
        {
            sendMotor(0);
        }
        else
        {
            DM4310::dm4310Motors[0].enableMotor();
        }

        vTaskDelay(1);

        if (DM4310::dm4310Motors[1].isConnected())
        {
            sendMotor(1);
        }
        else
        {
            DM4310::dm4310Motors[1].enableMotor();
        }
        // sendMotor(0);
        // sendMotor(1);
        // DM4310::dm4310Motors[1].enableMotor();
        // DM4310::dm4310Motors[0].enableMotor();

        // for (int i = 0; i < 2; i++)
        //     if (DM4310::dm4310Motors[i].getError() == MotorERR::LOSE_CONNECTION)
        //     {
        //         DM4310::dm4310Motors[i].clearErr();
        //         vTaskDelay(1);
        //         DM4310::dm4310Motors[i].enableMotor();
        //     }
        // xSemaphoreGive(motorUpdateMutex);  // schedule sync with motor update task
        motorUpdate(NULL);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void init()
{
    // FDCAN_FilterTypeDef DM4310Filter = {
    //     0,                      // filterID HI
    //     0,                      // filterID LO
    //     0,                      // filterMask HI
    //     0,                      // filterMask LO
    //     0,                      // FIFO assignment
    //     FDCAN_FILTER_TO_RXFIFO0,       // filterBank number
    //     CAN_FILTERMODE_IDMASK,  // filter mode
    //     CAN_FILTERSCALE_16BIT,  // filter size
    //     CAN_FILTER_ENABLE,      // ENABLE or DISABLE
    //     0                       // Slave start bank
    // };
    FDCAN_FilterTypeDef DM4310Filter = {FDCAN_STANDARD_ID,  // IdType
                                        0,                  // FilterIndex
                                        FDCAN_FILTER_MASK,
                                        FDCAN_FILTER_TO_RXFIFO0,
                                        0,
                                        0};

    configASSERT(HAL_FDCAN_ConfigFilter(&DM4310_CAN, &DM4310Filter) == HAL_OK);
    // configASSERT(HAL_FDCAN_ActivateNotification(&DM4310_CAN, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, (uint32_t)NULL) == HAL_OK);
    // configASSERT(HAL_FDCAN_RegisterRxFifo0Callback(&DM4310_CAN, DM4310::decodeFeedback) == HAL_OK);
    // configASSERT(HAL_FDCAN_Start(&DM4310_CAN) == HAL_OK);
    // motorUpdateMutex = xSemaphoreCreateBinaryStatic(&motorUpdateMutexBuffer);
    // xSemaphoreGive(motorUpdateMutex);
    DM4310::dm4310Motors[0].setID(0);
    DM4310::dm4310Motors[1].setID(1);
    DM4310::dm4310Motors[2].setID(2);
    // xTaskCreateStatic(motorUpdate, "MotorUpdate", 128, NULL, 10, uxMotorUpdateTaskStack, &xMotorUpdateTaskTCB);
    xTaskCreateStatic(sendMotorTask, "dm4310Motorsend", 128, NULL, 10, uxdm4310MotorsendTaskStack, &xdm4310MotorsendTaskTCB);
}

}  // namespace DM4310

#endif