
#include "CAN.hpp"
#if USE_CAN
#include "DM4310.hpp"
#include "FreeRTOS.h"
#include "M3508.hpp"
#include "math.h"
// #include "stm32g4xx_hal.h"
#include "task.h"

#ifdef HAL_CAN_MODULE_ENABLED

namespace CAN
{
void callback(CAN_HandleTypeDef *)
{
    while (HAL_CAN_GetRxFifoFillLevel(&CAN_Handler, CAN_RX_FIFO0))
    {
        uint32_t ulOriginalBASEPRI = taskENTER_CRITICAL_FROM_ISR();
        static volatile CAN_RxHeaderTypeDef canRxHeader;
        canRxHeader.StdId = 0;
        static volatile uint8_t RxBuffer[8];
        HAL_CAN_GetRxMessage(&CAN_Handler, CAN_RX_FIFO0, (CAN_RxHeaderTypeDef *)&canRxHeader, (uint8_t *)&RxBuffer);
        if (canRxHeader.StdId >= 0x201 && canRxHeader.StdId <= 0x20B)
        {
            M3508::decodeFeedback(*(CAN_RxHeaderTypeDef *)&canRxHeader, (uint8_t *)RxBuffer);
        }
        else
        {
            DM4310::decodeFeedback(*(CAN_RxHeaderTypeDef *)&canRxHeader, (uint8_t *)RxBuffer);
        }
        taskEXIT_CRITICAL_FROM_ISR(ulOriginalBASEPRI);
    }
}

void errorCallback(CAN_HandleTypeDef *, uint32_t)
{
    // HAL_FDCAN_DeInit(&hfdcan1);
    // HAL_FDCAN_Init(&hfdcan1);
    // __HAL_UNLOCK(&hfdcan1);
    // hfdcan1.State = HAL_FDCAN_STATE_READY;
    // HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, NULL) == HAL_OK;
    // HAL_FDCAN_RegisterRxFifo0Callback(&hfdcan1, callback) == HAL_OK;
    // HAL_FDCAN_RegisterErrorStatusCallback(&hfdcan1, errorCallback) == HAL_OK;
    // HAL_FDCAN_Start(&hfdcan1) == HAL_OK;
}

void init()
{
    CAN_FilterTypeDef CAN1Filter = {
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

    configASSERT(HAL_CAN_ConfigFilter(&CAN_Handler, &CAN1Filter) == HAL_OK);

        // configASSERT(HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_ERROR_LOGGING_OVERFLOW, NULL) == HAL_OK);
        configASSERT(HAL_CAN_RegisterCallback(&CAN_Handler, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, callback) == HAL_OK);
    // configASSERT(HAL_FDCAN_RegisterErrorStatusCallback(&hfdcan1, errorCallback) == HAL_OK);
    configASSERT(HAL_CAN_Start(&CAN_Handler) == HAL_OK);

       configASSERT(HAL_CAN_ActivateNotification(&CAN_Handler, CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK)
}
}  // namespace CAN

#elif defined HAL_FDCAN_MODULE_ENABLED

namespace FDCAN
{
void callback(FDCAN_HandleTypeDef *, uint32_t)
{
    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0))
    {
        uint32_t ulOriginalBASEPRI = taskENTER_CRITICAL_FROM_ISR();
        static volatile FDCAN_RxHeaderTypeDef canRxHeader;
        canRxHeader.Identifier = 0;
        static volatile uint8_t RxBuffer[8];
        HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, (FDCAN_RxHeaderTypeDef *)&canRxHeader, (uint8_t *)&RxBuffer);
        if (canRxHeader.Identifier >= 0x201 && canRxHeader.Identifier <= 0x20B)
        {
            M3508::decodeFeedback(*(FDCAN_RxHeaderTypeDef *)&canRxHeader, (uint8_t *)RxBuffer);
        }
        else
        {
            DM4310::decodeFeedback(*(FDCAN_RxHeaderTypeDef *)&canRxHeader, (uint8_t *)RxBuffer);
        }
        taskEXIT_CRITICAL_FROM_ISR(ulOriginalBASEPRI);
    }
}

void errorCallback(FDCAN_HandleTypeDef *, uint32_t)
{
    // HAL_FDCAN_DeInit(&hfdcan1);
    // HAL_FDCAN_Init(&hfdcan1);
    // __HAL_UNLOCK(&hfdcan1);
    // hfdcan1.State = HAL_FDCAN_STATE_READY;
    // HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, NULL) == HAL_OK;
    // HAL_FDCAN_RegisterRxFifo0Callback(&hfdcan1, callback) == HAL_OK;
    // HAL_FDCAN_RegisterErrorStatusCallback(&hfdcan1, errorCallback) == HAL_OK;
    // HAL_FDCAN_Start(&hfdcan1) == HAL_OK;
}

void init()
{
    configASSERT(HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, NULL) == HAL_OK)
        // configASSERT(HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_ERROR_LOGGING_OVERFLOW, NULL) == HAL_OK);
        configASSERT(HAL_FDCAN_RegisterRxFifo0Callback(&hfdcan1, callback) == HAL_OK);
    // configASSERT(HAL_FDCAN_RegisterErrorStatusCallback(&hfdcan1, errorCallback) == HAL_OK);
    configASSERT(HAL_FDCAN_Start(&hfdcan1) == HAL_OK);
}
}  // namespace FDCAN

#endif
#endif
