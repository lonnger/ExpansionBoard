/**
 * @file DR16.cpp
 * @author JIANG Yicheng (EthenJ@outlook.sg)
 * @brief
 * @version 0.2
 * @date 2022-12-26
 *
 * @copyright HKUST Enterprize Dotech2023
 */

#include "DR16.hpp"

#if USE_DR16 && (defined(HAL_UART_MODULE_ENABLED) || defined(HAL_USART_MODULE_ENABLED))

#include "FreeRTOS.h"
#include "main.h"
#include "timers.h"

/* DR16 receive timeout: 20 ms */
#define DR16_RECEIVE_TIMEOUT pdMS_TO_TICKS(20)

/* DR16 connection timeout: 200 ms */
#define DR16_CONNECTION_TIMEOUT pdMS_TO_TICKS(200)

/* DR16 fail count: 200 / 20 */
#define DR16_FAIL_COUNT DR16_CONNECTION_TIMEOUT / DR16_RECEIVE_TIMEOUT

namespace Core
{
namespace Drivers
{
namespace DR16
{

struct RxBuf
{
    uint16_t rc_ch0 : 11;
    uint16_t rc_ch1 : 11;
    uint16_t rc_ch2 : 11;
    uint16_t rc_ch3 : 11;
    uint16_t rc_s1 : 2;
    uint16_t rc_s2 : 2;

    int16_t mouse_x;       //!< Byte 6,7
    int16_t mouse_y;       //!< Byte 8,9
    int16_t mouse_z;       //!< Byte 10,11
    uint8_t mouse_pressL;  //!< Byte 12
    uint8_t mouse_pressR;  //!< Byte 13

    uint16_t key;          //!< Byte 14,15

    uint16_t rc_ch4 : 11;  //!< Byte 16

    uint8_t resv : 5;
} __attribute__((packed, aligned(1)));  // 18 bytes in total

// __DMA_BUFFER_ATTRIBUTE static volatile RxBuf rxBuf;  // receive buffer
static volatile RxBuf rxBuf;                         // receive buffer
static volatile RcData rcData;                       // rc data

static UART_HandleTypeDef *DR16_UART_DRIVER = NULL;  // UART driver

const volatile RcData &getRcData() { return rcData; }

bool isConnected() { return rcData.isConnected; }

/**
 * @brief Reset the DR16 data, automatically called when DR16 is disconnected
 */
static inline void DR16Reset()
{
    rcData.isConnected  = false;
    rcData.rc.ch0       = 1024U;
    rcData.rc.ch1       = 1024U;
    rcData.rc.ch2       = 1024U;
    rcData.rc.ch3       = 1024U;
    rcData.rc.s1        = 0U;
    rcData.rc.s2        = 0U;
    rcData.rc.ch4       = 1024U;
    rcData.mouse.x      = 0;
    rcData.mouse.y      = 0;
    rcData.mouse.z      = 0;
    rcData.mouse.pressL = false;
    rcData.mouse.pressR = false;
    rcData.key          = 0U;
}

static volatile uint32_t DR16TimeoutCnt = 0;     // timeout counter
static TimerHandle_t DR16TimeoutTimer   = NULL;  // timeout timer
static StaticTimer_t DR16TimeoutTimerBuf;
static void DR16TimeoutCallback(TimerHandle_t)
{
    if (DR16TimeoutCnt++ >= DR16_FAIL_COUNT)
    {
        DR16TimeoutCnt = DR16_FAIL_COUNT;  // prevent overflow

        /* Critical section enter */
        portENTER_CRITICAL();                                                              // enter critical section

        DR16Reset();                                                                       // reset rcData (fail-safe)

        HAL_UARTEx_ReceiveToIdle_DMA(DR16_UART_DRIVER, (uint8_t *)&rxBuf, sizeof(rxBuf));  // restart DMA transfer
        __HAL_DMA_DISABLE_IT(DR16_UART_DRIVER->hdmarx, DMA_IT_HT);                         // disable half transfer interrupt

        portEXIT_CRITICAL();                                                               // exit critical section
        /* Critical section exit */

        // timer auto-reload is enabled, no need to restart here
    }
}

/**
 * @brief DR16 error callback
 */
static void DR16ErrorCallback(UART_HandleTypeDef *)
{
    /* Critical section enter */
    UBaseType_t uxSavedInterruptStatus = portSET_INTERRUPT_MASK_FROM_ISR();            // enter critical section from ISR

    HAL_UART_Abort(DR16_UART_DRIVER);                                                  // abort current DMA transfer
    HAL_UARTEx_ReceiveToIdle_DMA(DR16_UART_DRIVER, (uint8_t *)&rxBuf, sizeof(rxBuf));  // restart DMA transfer
    __HAL_DMA_DISABLE_IT(DR16_UART_DRIVER->hdmarx, DMA_IT_HT);                         // disable half transfer interrupt

    portCLEAR_INTERRUPT_MASK_FROM_ISR(uxSavedInterruptStatus);                         // exit critical section from ISR
    /* Critical section exit */

    // timer auto-reload is enabled, no need to restart here
}

static inline bool DR16ValidateChannel(uint16_t ch) { return ch >= CH_VALUE_MIN - 10 && ch <= CH_VALUE_MAX + 10; }

static void DR16CompleteCallback(UART_HandleTypeDef *, uint16_t size)
{
    /* Critical section enter */
    UBaseType_t uxSavedInterruptStatus = portSET_INTERRUPT_MASK_FROM_ISR();  // enter critical section from ISR

    if (size == sizeof(RxBuf) && DR16ValidateChannel(rxBuf.rc_ch0) && DR16ValidateChannel(rxBuf.rc_ch1) && DR16ValidateChannel(rxBuf.rc_ch2) &&
        DR16ValidateChannel(rxBuf.rc_ch3) && rxBuf.rc_s1 != 0 && rxBuf.rc_s2 != 0 && rxBuf.mouse_pressL <= 1 && rxBuf.mouse_pressR <= 1)
    {
        DR16TimeoutCnt = 0;  // reset timeout counter

        /* Copy rcData */
        rcData.rc.ch0       = rxBuf.rc_ch0;
        rcData.rc.ch1       = rxBuf.rc_ch1;
        rcData.rc.ch2       = rxBuf.rc_ch2;
        rcData.rc.ch3       = rxBuf.rc_ch3;
        rcData.rc.ch4       = rxBuf.rc_ch4;
        rcData.rc.s1        = rxBuf.rc_s1;
        rcData.rc.s2        = rxBuf.rc_s2;
        rcData.mouse.x      = rxBuf.mouse_x;
        rcData.mouse.y      = rxBuf.mouse_y;
        rcData.mouse.z      = rxBuf.mouse_z;
        rcData.mouse.pressL = rxBuf.mouse_pressL == 0 ? false : true;
        rcData.mouse.pressR = rxBuf.mouse_pressR == 0 ? false : true;
        rcData.key          = rxBuf.key;

        rcData.isConnected = true;

        HAL_UARTEx_ReceiveToIdle_DMA(DR16_UART_DRIVER, (uint8_t *)&rxBuf, sizeof(rxBuf));  // restart DMA transfer
        __HAL_DMA_DISABLE_IT(DR16_UART_DRIVER->hdmarx, DMA_IT_HT);                         // disable half transfer interrupt

        portCLEAR_INTERRUPT_MASK_FROM_ISR(uxSavedInterruptStatus);                         // exit critical section from ISR
        /* Critical section exit */

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xTimerResetFromISR(DR16TimeoutTimer, &xHigherPriorityTaskWoken);  // restart timeout timer from ISR
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);                     // yield if necessary
    }
    else
    {
        portCLEAR_INTERRUPT_MASK_FROM_ISR(uxSavedInterruptStatus);  // exit critical section from ISR
        /* Critical section exit */

        DR16ErrorCallback(DR16_UART_DRIVER);  // some error occurred ? call error callback
    }
}

static_assert(sizeof(RxBuf) == 18, "DR16 Rx Buffer Size Error");
void init()
{
    DR16_UART_DRIVER = &DR16_UART;           // get user defined UART driver

    configASSERT(DR16TimeoutTimer == NULL);  // DR16 can only be initialized once
    configASSERT(DR16_UART_DRIVER->hdmarx != NULL);
    DR16TimeoutTimer =
        xTimerCreateStatic("dr16", DR16_RECEIVE_TIMEOUT, pdTRUE, NULL, DR16TimeoutCallback, &DR16TimeoutTimerBuf);  // create timeout timer

    configASSERT(HAL_UART_RegisterRxEventCallback(DR16_UART_DRIVER, DR16CompleteCallback) == HAL_OK);  // register uart rx complete callback
    configASSERT(HAL_UART_RegisterCallback(DR16_UART_DRIVER, HAL_UART_ERROR_CB_ID, DR16ErrorCallback) == HAL_OK);  // register uart error callback

    /* Critical section enter */
    portENTER_CRITICAL();                                                              // enter critical section

    HAL_UARTEx_ReceiveToIdle_DMA(DR16_UART_DRIVER, (uint8_t *)&rxBuf, sizeof(rxBuf));  // start DMA transfer
    __HAL_DMA_DISABLE_IT(DR16_UART_DRIVER->hdmarx, DMA_IT_HT);                         // disable half transfer interrupt

    portEXIT_CRITICAL();                                                               // exit critical section
    /* Critical section exit */

    configASSERT(xTimerStart(DR16TimeoutTimer, 20) == pdPASS);  // start timeout timer
}

}  // namespace DR16
}  // namespace Drivers
}  // namespace Core

#endif