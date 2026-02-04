/**
 * @file FutabaSBUS.cpp
 * @author Will
 * @brief
 * @version 0.1
 * @date 2024-03-08
 *
 * @copyright HKCRC 2024
 */

#pragma once

#include "appConfig.hpp"
#ifndef USE_FutabaSBUS
#define USE_FutabaSBUS 0
#endif

#if USE_FutabaSBUS && (defined(HAL_UART_MODULE_ENABLED) || defined(HAL_USART_MODULE_ENABLED))

#ifndef FutabaSBUS_UART
#define FutabaSBUS_UART huart4
#endif

#include "main.h"
#include "usart.h"

namespace Core
{
namespace Drivers
{
namespace FutabaSBUS
{

/* ----------------------- RC Channel Properties---------------------------- */
constexpr uint16_t CH_VALUE_MIN       = 364;                                  // the minimum value of a channel
constexpr uint16_t CH_VALUE_MID       = 1024;                                 // the middle value of a channel
constexpr uint16_t CH_VALUE_MAX       = 1684;                                 // the maximum value of a channel
constexpr uint16_t CH_VALUE_ABS_RANGE = ((CH_VALUE_MAX - CH_VALUE_MIN) / 2);  // the absolute range of a channel

/**
 * @brief RC data structure
 */
struct RcDataSBUS
{
    /**
     * @brief remote controller channel data
     */
    struct
    {
        uint16_t ch0  = 1024U;
        uint16_t ch1  = 1024U;
        uint16_t ch2  = 1024U;
        uint16_t ch3  = 1024U;
        uint16_t ch4  = 1024U;
        uint16_t ch5  = 1024U;
        uint16_t ch6  = 1024U;
        uint16_t ch7  = 1024U;
        uint16_t ch8  = 1024U;
        uint16_t ch9  = 1024U;
        uint16_t ch10 = 1024U;
        uint16_t ch11 = 1024U;
        uint16_t ch12 = 1024U;
        uint16_t ch13 = 1024U;
        uint16_t ch14 = 1024U;
        uint16_t ch15 = 1024U;
    } rc;

    /**
     * @brief connection status
     */
    bool isConnected = false;
};

/**
 * @brief Get the FutabaSBUS data
 *
 * @return const volatile RcData &
 */
const volatile RcDataSBUS &getRcDataSBUS();

/**
 * @brief Check if the FutabaSBUS is connected
 *
 * @return true
 * @return false
 */
bool isConnected();

/**
 * @brief initialize the FutabaSBUS, call this function before using the FutabaSBUS
 */
void init();

}  // namespace FutabaSBUS
}  // namespace Drivers
}  // namespace Core

#endif