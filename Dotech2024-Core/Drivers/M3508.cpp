#include "M3508.hpp"

#if USE_M3508 && defined(HAL_CAN_MODULE_ENABLED)

#include "FreeRTOS.h"
#include "cmsis_compiler.h"
#include "math.h"
#include "task.h"

#define M3508_TIMEOUT pdMS_TO_TICKS(200)

namespace M3508
{
M3508::M3508()
    : rawEncoder(0),
      lastRawEncoder(0),
      position(0),
      rpm(0),
      actualCurrent(0),
      currentLimit(16383),
      temperature(0),
      rotaryCnt(0),
      positionOffset(0),
      disconnectCnt(200),
      receiveCnt(0),
      connected(false)
{
}

M3508 M3508::motors[11];

uint16_t M3508::getRawEncoder() const { return this->rawEncoder; }

float M3508::getPosition() const { return this->position; }

void M3508::setPosition(float setPosition)
{
    uint32_t ulOriginalBASEPRI = 0;
    if (__get_IPSR())
        ulOriginalBASEPRI = taskENTER_CRITICAL_FROM_ISR();
    else
        taskENTER_CRITICAL();

    this->positionOffset = (int16_t)((setPosition - this->position) / (float)M_PI / 2 * 8192.0f);

    if (__get_IPSR())
        taskEXIT_CRITICAL_FROM_ISR(ulOriginalBASEPRI);
    else
        taskEXIT_CRITICAL();
}

int16_t M3508::getRPM() const { return this->rpm; }

int16_t M3508::getActualCurrent() const { return this->actualCurrent; }

int16_t M3508::getOutputCurrent() const { return this->setCurrent; }

void M3508::setOutputCurrent(int32_t current)
{
    if (current > this->currentLimit)
        current = this->currentLimit;
    else if (current < -this->currentLimit)
        current = -this->currentLimit;
    this->setCurrent = current;
}

void M3508::setCurrentLimit(uint16_t current)
{
    configASSERT(current <= 30000);
    this->currentLimit = current;
}

uint8_t M3508::getTemperature() const { return this->temperature; }

uint32_t M3508::getReveiceCount() const { return this->receiveCnt; }

bool M3508::isConnected() const { return this->connected; }

M3508 &getMotor(uint32_t canid)
{
    canid -= 0x201;
    configASSERT(canid < 11);
    return M3508::motors[canid];
}

void sendMotorGroup(uint32_t group)
{
    configASSERT(group < 3);
    uint32_t ulOriginalBASEPRI = 0;
    if (__get_IPSR())
        ulOriginalBASEPRI = taskENTER_CRITICAL_FROM_ISR();
    else
        taskENTER_CRITICAL();

    int8_t canTxData[8];
    static uint32_t canMailBox[3];
    static volatile uint32_t canLastSend = 0;
    constexpr uint32_t ids[]             = {0x200, 0x1FF, 0x2FF};
    // FDCAN_TxHeaderTypeDef canTxHeader = {
    //     0x200,         // Identifier
    //     0,             // ExtId
    //     CAN_ID_STD,    // IDE
    //     CAN_RTR_DATA,  // RTR
    //     8,             // DLC
    //     DISABLE        // Transmit Global Time
    // };
    // FDCAN_TxHeaderTypeDef canTxHeader = {
    //     0x200, FDCAN_STANDARD_ID, FDCAN_DATA_FRAME, FDCAN_DLC_BYTES_8, FDCAN_ESI_ACTIVE, FDCAN_BRS_OFF, FDCAN_CLASSIC_CAN, FDCAN_NO_TX_EVENTS, 0};
    CAN_TxHeaderTypeDef canTxHeader = {
        0x200,         // Identifier
        0,             // ExtId
        0,             // IDE
        CAN_RTR_DATA,  // RTR
        8,             // DLC
        DISABLE        // Transmit Global Time
    };
    if (group != 2)
    {
        for (uint32_t i = 0; i < 4; i++)
        {
            canTxData[2 * i]     = M3508::motors[i + 4 * group].setCurrent >> 8;
            canTxData[2 * i + 1] = M3508::motors[i + 4 * group].setCurrent & 0xFF;
        }
    }
    else
    {
        for (uint32_t i = 0; i < 3; i++)
        {
            canTxData[2 * i]     = M3508::motors[i + 8].setCurrent >> 8;
            canTxData[2 * i + 1] = M3508::motors[i + 8].setCurrent & 0xFF;
        }
        canTxData[6] = 0;
        canTxData[7] = 0;
    }

    canTxHeader.StdId = ids[group];
    if (HAL_CAN_GetTxMailboxesFreeLevel(&M3508_CAN) == 0)
        HAL_CAN_AbortTxRequest(&M3508_CAN, canMailBox[(canLastSend + 1) % 3]);
    HAL_CAN_AddTxMessage(&M3508_CAN, &canTxHeader, (uint8_t *)canTxData, &canMailBox[canLastSend]);
    canLastSend = (canLastSend + 1) % 3;

    if (__get_IPSR())
        taskEXIT_CRITICAL_FROM_ISR(ulOriginalBASEPRI);
    else
        taskEXIT_CRITICAL();
}

static StackType_t uxMotorUpdateTaskStack[128];
static StaticTask_t xMotorUpdateTaskTCB;
void motorUpdate(void *)
{
    static TickType_t xLastWakeTime = xTaskGetTickCount();
    while (true)
    {
        for (uint32_t motorNum = 0; motorNum < 11; motorNum++)
            if (M3508::motors[motorNum].disconnectCnt++ > M3508_TIMEOUT)
            {
                M3508::motors[motorNum].connected     = false;
                M3508::motors[motorNum].disconnectCnt = M3508_TIMEOUT;
            }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
}

void M3508::decodeFeedback(CAN_HandleTypeDef *, uint32_t rxFifoITs)
{
    while (HAL_CAN_GetRxFifoFillLevel(&M3508_CAN, CAN_RX_FIFO0))
    {
        CAN_RxHeaderTypeDef M3508canRxHeader;
        M3508canRxHeader.StdId = 0;
        static uint8_t M3508RxBuffer[8];
        HAL_CAN_GetRxMessage(&M3508_CAN, CAN_RX_FIFO0, &M3508canRxHeader, (uint8_t *)&M3508RxBuffer);
        if (M3508canRxHeader.StdId >= 0x201 && M3508canRxHeader.StdId <= 0x20B)
        {
            uint32_t ulOriginalBASEPRI = taskENTER_CRITICAL_FROM_ISR();
            uint32_t id                = M3508canRxHeader.StdId - 0x201;

            M3508::motors[id].lastRawEncoder = M3508::motors[id].rawEncoder;

            M3508::motors[id].rawEncoder = M3508RxBuffer[0] << 8 | M3508RxBuffer[1];

            M3508::motors[id].rpm = M3508RxBuffer[2] << 8 | M3508RxBuffer[3];

            M3508::motors[id].actualCurrent = M3508RxBuffer[4] << 8 | M3508RxBuffer[5];

            M3508::motors[id].temperature = M3508RxBuffer[6];

            int32_t posDiff = M3508::motors[id].rawEncoder - M3508::motors[id].lastRawEncoder;
            if (posDiff > 4096)
                M3508::motors[id].rotaryCnt--;
            else if (posDiff < -4096)
                M3508::motors[id].rotaryCnt++;
            M3508::motors[id].position =
                (float)(M3508::motors[id].rawEncoder + M3508::motors[id].rotaryCnt * 8192 + M3508::motors[id].positionOffset) / 8192.0f *
                (float)M_PI * 2;
            M3508::motors[id].disconnectCnt = 0;
            M3508::motors[id].receiveCnt++;
            M3508::motors[id].connected = true;
            taskEXIT_CRITICAL_FROM_ISR(ulOriginalBASEPRI);
        }
    }
}

void decodeFeedback(CAN_RxHeaderTypeDef &M3508canRxHeader, uint8_t M3508RxBuffer[8])
{
    uint32_t id = M3508canRxHeader.StdId - 0x201;

    M3508::motors[id].lastRawEncoder = M3508::motors[id].rawEncoder;

    M3508::motors[id].rawEncoder = M3508RxBuffer[0] << 8 | M3508RxBuffer[1];

    M3508::motors[id].rpm = M3508RxBuffer[2] << 8 | M3508RxBuffer[3];

    M3508::motors[id].actualCurrent = M3508RxBuffer[4] << 8 | M3508RxBuffer[5];

    M3508::motors[id].temperature = M3508RxBuffer[6];

    int32_t posDiff = M3508::motors[id].rawEncoder - M3508::motors[id].lastRawEncoder;
    if (posDiff > 4096)
        M3508::motors[id].rotaryCnt--;
    else if (posDiff < -4096)
        M3508::motors[id].rotaryCnt++;
    M3508::motors[id].position =
        (float)(M3508::motors[id].rawEncoder + M3508::motors[id].rotaryCnt * 8192 + M3508::motors[id].positionOffset) / 8192.0f * (float)M_PI * 2;
    M3508::motors[id].disconnectCnt = 0;
    M3508::motors[id].receiveCnt++;
    M3508::motors[id].connected = true;
}

void init()
{
    // FDCAN_FilterTypeDef M3508Filter = {FDCAN_STANDARD_ID,
    //                                    0,  // filterID HI
    //                                    FDCAN_FILTER_MASK,
    //                                    FDCAN_FILTER_TO_RXFIFO0,
    //                                    0x201,  // filterID LO
    //                                    0x20B};

    // configASSERT(HAL_FDCAN_ConfigFilter(&M3508_CAN, &M3508Filter) == HAL_OK);
    // configASSERT(HAL_FDCAN_ActivateNotification(&M3508_CAN, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, NULL) == HAL_OK);
    // configASSERT(HAL_FDCAN_RegisterRxFifo0Callback(&M3508_CAN, M3508::decodeFeedback) == HAL_OK);
    // configASSERT(HAL_FDCAN_Start(&M3508_CAN) == HAL_OK);
    xTaskCreateStatic(motorUpdate, "MotorUpdate", 128, NULL, 14, uxMotorUpdateTaskStack, &xMotorUpdateTaskTCB);
}

}  // namespace M3508

#elif defined HAL_FDCAN_MODULE_ENABLED

#include "fdcan.h"

namespace M3508
{
M3508::M3508()
    : rawEncoder(0),
      lastRawEncoder(0),
      position(0),
      rpm(0),
      actualCurrent(0),
      currentLimit(16383),
      temperature(0),
      rotaryCnt(0),
      positionOffset(0),
      disconnectCnt(200),
      receiveCnt(0),
      connected(false)
{
}

M3508 M3508::motors[11];

uint16_t M3508::getRawEncoder() const { return this->rawEncoder; }

float M3508::getPosition() const { return this->position; }

void M3508::setPosition(float setPosition)
{
    uint32_t ulOriginalBASEPRI = 0;
    if (__get_IPSR())
        ulOriginalBASEPRI = taskENTER_CRITICAL_FROM_ISR();
    else
        taskENTER_CRITICAL();

    this->positionOffset = (int16_t)((setPosition - this->position) / (float)M_PI / 2 * 8192.0f);

    if (__get_IPSR())
        taskEXIT_CRITICAL_FROM_ISR(ulOriginalBASEPRI);
    else
        taskEXIT_CRITICAL();
}

int16_t M3508::getRPM() const { return this->rpm; }

int16_t M3508::getActualCurrent() const { return this->actualCurrent; }

int16_t M3508::getOutputCurrent() const { return this->setCurrent; }

void M3508::setOutputCurrent(int32_t current)
{
    if (current > this->currentLimit)
        current = this->currentLimit;
    else if (current < -this->currentLimit)
        current = -this->currentLimit;
    this->setCurrent = current;
}

void M3508::setCurrentLimit(uint16_t current)
{
    configASSERT(current <= 30000);
    this->currentLimit = current;
}

uint8_t M3508::getTemperature() const { return this->temperature; }

uint32_t M3508::getReveiceCount() const { return this->receiveCnt; }

bool M3508::isConnected() const { return this->connected; }

M3508 &getMotor(uint32_t canid)
{
    canid -= 0x201;
    configASSERT(canid < 11);
    return M3508::motors[canid];
}

void sendMotorGroup(uint32_t group)
{
    configASSERT(group < 3);
    uint32_t ulOriginalBASEPRI = 0;
    if (__get_IPSR())
        ulOriginalBASEPRI = taskENTER_CRITICAL_FROM_ISR();
    else
        taskENTER_CRITICAL();

    int8_t canTxData[8];
    static volatile uint32_t canMailBox[3];
    static volatile uint32_t canLastSend = 0;
    constexpr uint32_t ids[]             = {0x200, 0x1FF, 0x2FF};
    // FDCAN_TxHeaderTypeDef canTxHeader = {
    //     0x200,         // Identifier
    //     0,             // ExtId
    //     CAN_ID_STD,    // IDE
    //     CAN_RTR_DATA,  // RTR
    //     8,             // DLC
    //     DISABLE        // Transmit Global Time
    // };
    FDCAN_TxHeaderTypeDef canTxHeader = {
        0x200, FDCAN_STANDARD_ID, FDCAN_DATA_FRAME, FDCAN_DLC_BYTES_8, FDCAN_ESI_ACTIVE, FDCAN_BRS_OFF, FDCAN_CLASSIC_CAN, FDCAN_NO_TX_EVENTS, 0};
    if (group != 2)
    {
        for (uint32_t i = 0; i < 4; i++)
        {
            canTxData[2 * i]     = M3508::motors[i + 4 * group].setCurrent >> 8;
            canTxData[2 * i + 1] = M3508::motors[i + 4 * group].setCurrent & 0xFF;
        }
    }
    else
    {
        for (uint32_t i = 0; i < 3; i++)
        {
            canTxData[2 * i]     = M3508::motors[i + 8].setCurrent >> 8;
            canTxData[2 * i + 1] = M3508::motors[i + 8].setCurrent & 0xFF;
        }
        canTxData[6] = 0;
        canTxData[7] = 0;
    }

    canTxHeader.Identifier = ids[group];
    if (HAL_FDCAN_GetTxFifoFreeLevel(&M3508_CAN) == 0)
        HAL_FDCAN_AbortTxRequest(&M3508_CAN, canMailBox[(canLastSend + 1) % 3]);
    HAL_FDCAN_AddMessageToTxFifoQ(&M3508_CAN, &canTxHeader, (uint8_t *)canTxData);
    canLastSend = (canLastSend + 1) % 3;

    if (__get_IPSR())
        taskEXIT_CRITICAL_FROM_ISR(ulOriginalBASEPRI);
    else
        taskEXIT_CRITICAL();
}

static StackType_t uxMotorUpdateTaskStack[128];
static StaticTask_t xMotorUpdateTaskTCB;
void motorUpdate(void *)
{
    static TickType_t xLastWakeTime = xTaskGetTickCount();
    while (true)
    {
        for (uint32_t motorNum = 0; motorNum < 11; motorNum++)
            if (M3508::motors[motorNum].disconnectCnt++ > M3508_TIMEOUT)
            {
                M3508::motors[motorNum].connected     = false;
                M3508::motors[motorNum].disconnectCnt = M3508_TIMEOUT;
            }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
}

void M3508::decodeFeedback(FDCAN_HandleTypeDef *, uint32_t rxFifoITs)
{
    while (HAL_FDCAN_GetRxFifoFillLevel(&M3508_CAN, FDCAN_RX_FIFO0))
    {
        FDCAN_RxHeaderTypeDef M3508canRxHeader;
        M3508canRxHeader.Identifier = 0;
        static uint8_t M3508RxBuffer[8];
        HAL_FDCAN_GetRxMessage(&M3508_CAN, FDCAN_RX_FIFO0, &M3508canRxHeader, (uint8_t *)&M3508RxBuffer);
        if (M3508canRxHeader.Identifier >= 0x201 && M3508canRxHeader.Identifier <= 0x20B)
        {
            uint32_t ulOriginalBASEPRI = taskENTER_CRITICAL_FROM_ISR();
            uint32_t id                = M3508canRxHeader.Identifier - 0x201;

            M3508::motors[id].lastRawEncoder = M3508::motors[id].rawEncoder;

            M3508::motors[id].rawEncoder = M3508RxBuffer[0] << 8 | M3508RxBuffer[1];

            M3508::motors[id].rpm = M3508RxBuffer[2] << 8 | M3508RxBuffer[3];

            M3508::motors[id].actualCurrent = M3508RxBuffer[4] << 8 | M3508RxBuffer[5];

            M3508::motors[id].temperature = M3508RxBuffer[6];

            int32_t posDiff = M3508::motors[id].rawEncoder - M3508::motors[id].lastRawEncoder;
            if (posDiff > 4096)
                M3508::motors[id].rotaryCnt--;
            else if (posDiff < -4096)
                M3508::motors[id].rotaryCnt++;
            M3508::motors[id].position =
                (float)(M3508::motors[id].rawEncoder + M3508::motors[id].rotaryCnt * 8192 + M3508::motors[id].positionOffset) / 8192.0f *
                (float)M_PI * 2;
            M3508::motors[id].disconnectCnt = 0;
            M3508::motors[id].receiveCnt++;
            M3508::motors[id].connected = true;
            taskEXIT_CRITICAL_FROM_ISR(ulOriginalBASEPRI);
        }
    }
}

void decodeFeedback(FDCAN_RxHeaderTypeDef &M3508canRxHeader, uint8_t M3508RxBuffer[8])
{
    uint32_t id = M3508canRxHeader.Identifier - 0x201;

    M3508::motors[id].lastRawEncoder = M3508::motors[id].rawEncoder;

    M3508::motors[id].rawEncoder = M3508RxBuffer[0] << 8 | M3508RxBuffer[1];

    M3508::motors[id].rpm = M3508RxBuffer[2] << 8 | M3508RxBuffer[3];

    M3508::motors[id].actualCurrent = M3508RxBuffer[4] << 8 | M3508RxBuffer[5];

    M3508::motors[id].temperature = M3508RxBuffer[6];

    int32_t posDiff = M3508::motors[id].rawEncoder - M3508::motors[id].lastRawEncoder;
    if (posDiff > 4096)
        M3508::motors[id].rotaryCnt--;
    else if (posDiff < -4096)
        M3508::motors[id].rotaryCnt++;
    M3508::motors[id].position =
        (float)(M3508::motors[id].rawEncoder + M3508::motors[id].rotaryCnt * 8192 + M3508::motors[id].positionOffset) / 8192.0f * (float)M_PI * 2;
    M3508::motors[id].disconnectCnt = 0;
    M3508::motors[id].receiveCnt++;
    M3508::motors[id].connected = true;
}

void init()
{
    // FDCAN_FilterTypeDef M3508Filter = {FDCAN_STANDARD_ID,
    //                                    0,  // filterID HI
    //                                    FDCAN_FILTER_MASK,
    //                                    FDCAN_FILTER_TO_RXFIFO0,
    //                                    0x201,  // filterID LO
    //                                    0x20B};

    // configASSERT(HAL_FDCAN_ConfigFilter(&M3508_CAN, &M3508Filter) == HAL_OK);
    // configASSERT(HAL_FDCAN_ActivateNotification(&M3508_CAN, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, NULL) == HAL_OK);
    // configASSERT(HAL_FDCAN_RegisterRxFifo0Callback(&M3508_CAN, M3508::decodeFeedback) == HAL_OK);
    // configASSERT(HAL_FDCAN_Start(&M3508_CAN) == HAL_OK);
    xTaskCreateStatic(motorUpdate, "MotorUpdate", 128, NULL, 14, uxMotorUpdateTaskStack, &xMotorUpdateTaskTCB);
}

}  // namespace M3508

#endif