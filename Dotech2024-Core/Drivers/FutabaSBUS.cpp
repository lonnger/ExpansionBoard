/**
 * @file FutabaSBUS.cpp
 * @author Will
 * @brief
 * @version 0.1
 * @date 2024-03-08
 *
 * @copyright HKCRC 2024
 */

#include "FutabaSBUS.hpp"

#if USE_FutabaSBUS && (defined(HAL_UART_MODULE_ENABLED) || defined(HAL_USART_MODULE_ENABLED))

#include "FreeRTOS.h"
#include "main.h"
#include "tim.h"
#include "timers.h"

/* FutabaSBUS receive timeout: 20 ms */
#define FutabaSBUS_RECEIVE_TIMEOUT pdMS_TO_TICKS(20)

/* FutabaSBUS connection timeout: 200 ms */
#define FutabaSBUS_CONNECTION_TIMEOUT pdMS_TO_TICKS(200)

/* FutabaSBUS fail count: 200 / 20 */
#define FutabaSBUS_FAIL_COUNT FutabaSBUS_CONNECTION_TIMEOUT / FutabaSBUS_RECEIVE_TIMEOUT

namespace Core
{
namespace Drivers
{
namespace FutabaSBUS
{

struct RxBufSBUS
{
    uint8_t buf[25] = {0};
};

// __DMA_BUFFER_ATTRIBUTE static volatile RxBuf rxBuf;  // receive buffer
static volatile RxBufSBUS rxBuf;                           // receive buffer
static volatile RcDataSBUS rcData;                         // rc data

static UART_HandleTypeDef *FutabaSBUS_UART_DRIVER = NULL;  // UART driver

const volatile RcDataSBUS &getRcDataSBUS() { return rcData; }

bool isConnected() { return rcData.isConnected; }

/**
 * @brief Reset the FutabaSBUS data, automatically called when FutabaSBUS is disconnected
 */
static inline void FutabaSBUSReset()
{
    rcData.isConnected = false;
    rcData.rc.ch0      = 1024U;
    rcData.rc.ch1      = 1024U;
    rcData.rc.ch2      = 1024U;
    rcData.rc.ch3      = 1024U;
    rcData.rc.ch4      = 1024U;
    rcData.rc.ch5      = 1024U;
    rcData.rc.ch6      = 1024U;
    rcData.rc.ch7      = 1024U;
    rcData.rc.ch8      = 1024U;
    rcData.rc.ch9      = 1024U;
    rcData.rc.ch10     = 1024U;
    rcData.rc.ch11     = 1024U;
    rcData.rc.ch12     = 1024U;
    rcData.rc.ch13     = 1024U;
    rcData.rc.ch14     = 1024U;
    rcData.rc.ch15     = 1024U;
}

static volatile uint32_t FutabaSBUSTimeoutCnt = 0;     // timeout counter
static TimerHandle_t FutabaSBUSTimeoutTimer   = NULL;  // timeout timer
static StaticTimer_t FutabaSBUSTimeoutTimerBuf;
static void FutabaSBUSTimeoutCallback(TimerHandle_t)
{
    if (FutabaSBUSTimeoutCnt++ >= FutabaSBUS_FAIL_COUNT)
    {
        FutabaSBUSTimeoutCnt = FutabaSBUS_FAIL_COUNT;  // prevent overflow

        /* Critical section enter */
        portENTER_CRITICAL();                                                                    // enter critical section

        FutabaSBUSReset();                                                                       // reset rcData (fail-safe)

        HAL_UARTEx_ReceiveToIdle_DMA(FutabaSBUS_UART_DRIVER, (uint8_t *)&rxBuf, sizeof(rxBuf));  // restart DMA transfer
        __HAL_DMA_DISABLE_IT(FutabaSBUS_UART_DRIVER->hdmarx, DMA_IT_HT);                         // disable half transfer interrupt

        portEXIT_CRITICAL();                                                                     // exit critical section
        /* Critical section exit */

        // timer auto-reload is enabled, no need to restart here
    }
}

/**
 * @brief FutabaSBUS error callback
 */
static void FutabaSBUSErrorCallback(UART_HandleTypeDef *)
{
    /* Critical section enter */
    UBaseType_t uxSavedInterruptStatus = portSET_INTERRUPT_MASK_FROM_ISR();                  // enter critical section from ISR

    HAL_UART_Abort(FutabaSBUS_UART_DRIVER);                                                  // abort current DMA transfer
    HAL_UARTEx_ReceiveToIdle_DMA(FutabaSBUS_UART_DRIVER, (uint8_t *)&rxBuf, sizeof(rxBuf));  // restart DMA transfer
    __HAL_DMA_DISABLE_IT(FutabaSBUS_UART_DRIVER->hdmarx, DMA_IT_HT);                         // disable half transfer interrupt

    portCLEAR_INTERRUPT_MASK_FROM_ISR(uxSavedInterruptStatus);                               // exit critical section from ISR
    /* Critical section exit */

    // timer auto-reload is enabled, no need to restart here
}

static inline bool FutabaSBUSValidateChannel(uint16_t ch) { return ch >= CH_VALUE_MIN - 10 && ch <= CH_VALUE_MAX + 10; }


uint8_t testTX[3] = {0x83, 0x69, 0x69};
void timerPeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    HAL_UART_Transmit_IT(FutabaSBUS_UART_DRIVER, testTX, sizeof(testTX));
    HAL_TIM_Base_Stop_IT(&htim6);
}

static void FutabaSBUSCompleteCallback(UART_HandleTypeDef *, uint16_t size)
{
    /* Critical section enter */
    UBaseType_t uxSavedInterruptStatus = portSET_INTERRUPT_MASK_FROM_ISR();  // enter critical section from ISR

    // if (size == sizeof(RxBufSBUS) && FutabaSBUSValidateChannel(rxBuf.rc_ch0) && FutabaSBUSValidateChannel(rxBuf.rc_ch1) &&
    //     FutabaSBUSValidateChannel(rxBuf.rc_ch2) && FutabaSBUSValidateChannel(rxBuf.rc_ch3) && FutabaSBUSValidateChannel(rxBuf.rc_ch4) &&
    //     FutabaSBUSValidateChannel(rxBuf.rc_ch5) && FutabaSBUSValidateChannel(rxBuf.rc_ch6) && FutabaSBUSValidateChannel(rxBuf.rc_ch7))
    // {
    if (rxBuf.buf[0] == 0x0F)
    {
        FutabaSBUSTimeoutCnt = 0;  // reset timeout counter
        __HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);
        HAL_TIM_Base_Start_IT(&htim6);

        /* Copy rcData */
        rcData.rc.ch0 = ((int16_t)rxBuf.buf[1] >> 0 | ((int16_t)rxBuf.buf[2] << 8)) & 0x07FF;
        rcData.rc.ch1 = ((int16_t)rxBuf.buf[2] >> 3 | ((int16_t)rxBuf.buf[3] << 5)) & 0x07FF;
        rcData.rc.ch2 = ((int16_t)rxBuf.buf[3] >> 6 | ((int16_t)rxBuf.buf[4] << 2) | (int16_t)rxBuf.buf[5] << 10) & 0x07FF;
        rcData.rc.ch3 = ((int16_t)rxBuf.buf[5] >> 1 | ((int16_t)rxBuf.buf[6] << 7)) & 0x07FF;
        rcData.rc.ch4 = ((int16_t)rxBuf.buf[6] >> 4 | ((int16_t)rxBuf.buf[7] << 4)) & 0x07FF;
        rcData.rc.ch5 = ((int16_t)rxBuf.buf[7] >> 7 | ((int16_t)rxBuf.buf[8] << 1) | (int16_t)rxBuf.buf[9] << 9) & 0x07FF;
        rcData.rc.ch6 = ((int16_t)rxBuf.buf[9] >> 2 | ((int16_t)rxBuf.buf[10] << 6)) & 0x07FF;
        rcData.rc.ch7 = ((int16_t)rxBuf.buf[10] >> 5 | ((int16_t)rxBuf.buf[11] << 3)) & 0x07FF;

        rcData.rc.ch8  = ((int16_t)rxBuf.buf[12] << 0 | ((int16_t)rxBuf.buf[13] << 8)) & 0x07FF;
        rcData.rc.ch9  = ((int16_t)rxBuf.buf[13] >> 3 | ((int16_t)rxBuf.buf[14] << 5)) & 0x07FF;
        rcData.rc.ch10 = ((int16_t)rxBuf.buf[14] >> 6 | ((int16_t)rxBuf.buf[15] << 2) | (int16_t)rxBuf.buf[16] << 10) & 0x07FF;
        rcData.rc.ch11 = ((int16_t)rxBuf.buf[16] >> 1 | ((int16_t)rxBuf.buf[17] << 7)) & 0x07FF;
        rcData.rc.ch12 = ((int16_t)rxBuf.buf[17] >> 4 | ((int16_t)rxBuf.buf[18] << 4)) & 0x07FF;
        rcData.rc.ch13 = ((int16_t)rxBuf.buf[20] >> 7 | ((int16_t)rxBuf.buf[21] << 1) | (int16_t)rxBuf.buf[22] << 9) & 0x07FF;
        rcData.rc.ch14 = ((int16_t)rxBuf.buf[20] >> 2 | ((int16_t)rxBuf.buf[21] << 6)) & 0x07FF;
        rcData.rc.ch15 = ((int16_t)rxBuf.buf[21] >> 5 | ((int16_t)rxBuf.buf[22] << 3)) & 0x07FF;

        rcData.isConnected = ((rxBuf.buf[23] >> 2) & 0x0001) ? false : true;
    }

    HAL_UARTEx_ReceiveToIdle_DMA(FutabaSBUS_UART_DRIVER, (uint8_t *)&rxBuf, sizeof(rxBuf));  // restart DMA transfer
    __HAL_DMA_DISABLE_IT(FutabaSBUS_UART_DRIVER->hdmarx, DMA_IT_HT);                         // disable half transfer interrupt

    portCLEAR_INTERRUPT_MASK_FROM_ISR(uxSavedInterruptStatus);                               // exit critical section from ISR
    /* Critical section exit */

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTimerResetFromISR(FutabaSBUSTimeoutTimer, &xHigherPriorityTaskWoken);  // restart timeout timer from ISR
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);                           // yield if necessary
    // }
    // else
    // {
    //     portCLEAR_INTERRUPT_MASK_FROM_ISR(uxSavedInterruptStatus);  // exit critical section from ISR
    //     /* Critical section exit */

    //     FutabaSBUSErrorCallback(FutabaSBUS_UART_DRIVER);  // some error occurred ? call error callback
    // }
}

static_assert(sizeof(RxBufSBUS) == 25, "FutabaSBUS Rx Buffer Size Error");
void init()
{
    FutabaSBUS_UART_DRIVER = &FutabaSBUS_UART;     // get user defined UART driver

    configASSERT(FutabaSBUSTimeoutTimer == NULL);  // FutabaSBUS can only be initialized once
    configASSERT(FutabaSBUS_UART_DRIVER->hdmarx != NULL);
    FutabaSBUSTimeoutTimer = xTimerCreateStatic(
        "FutabaSBUS", FutabaSBUS_RECEIVE_TIMEOUT, pdTRUE, NULL, FutabaSBUSTimeoutCallback, &FutabaSBUSTimeoutTimerBuf);  // create timeout timer

    configASSERT(HAL_UART_RegisterRxEventCallback(FutabaSBUS_UART_DRIVER, FutabaSBUSCompleteCallback) ==
                 HAL_OK);  // register uart rx complete callback
    configASSERT(HAL_UART_RegisterCallback(FutabaSBUS_UART_DRIVER, HAL_UART_ERROR_CB_ID, FutabaSBUSErrorCallback) ==
                 HAL_OK);  // register uart error callback

    /* Critical section enter */
    portENTER_CRITICAL();  // enter critical section

    HAL_TIM_RegisterCallback(&htim6, HAL_TIM_PERIOD_ELAPSED_CB_ID, timerPeriodElapsedCallback);

    HAL_UARTEx_ReceiveToIdle_DMA(FutabaSBUS_UART_DRIVER, (uint8_t *)&rxBuf, sizeof(rxBuf));  // start DMA transfer
    __HAL_DMA_DISABLE_IT(FutabaSBUS_UART_DRIVER->hdmarx, DMA_IT_HT);                         // disable half transfer interrupt

    portEXIT_CRITICAL();                                                                     // exit critical section
    /* Critical section exit */

    configASSERT(xTimerStart(FutabaSBUSTimeoutTimer, FutabaSBUS_RECEIVE_TIMEOUT) == pdPASS);  // start timeout timer
}

}  // namespace FutabaSBUS
}  // namespace Drivers
}  // namespace Core

#endif