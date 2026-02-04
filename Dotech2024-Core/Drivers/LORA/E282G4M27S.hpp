// /**
//  * @file E282G4M27S.hpp
//  * @author
//  * @brief
//  * @version 0.1
//  * @date 2022-10-25
//  *
//  * @copyright Dotech 2023
//  */

// #pragma once

// #include "Config.h"
// #include "FreeRTOS.h"
// #include "main.h"
// #include "sx1280.h"

// namespace Core
// {
// namespace Drivers
// {
// namespace E282G4M27S
// {

// /* ------------------------ joystick Properties---------------------------- */
// // this is different for every joystick
// constexpr uint16_t vX_VALUE_MIN        = 720;                                  // the minimum value of a channel
// constexpr uint16_t vX_VALUE_MID        = 1900;                                 // the middle value of a channel
// constexpr uint16_t vX_VALUE_MAX        = 3200;                                 // the maximum value of a channel
// constexpr uint16_t vX_LOWER_RESOLUTION = ((vX_VALUE_MID - vX_VALUE_MIN) / 7);  // the absolute range of a channel
// constexpr uint16_t vX_UPPER_RESOLUTION = ((vX_VALUE_MAX - vX_VALUE_MID) / 7);  // the absolute range of a channel
// constexpr uint16_t vY_VALUE_MIN        = 920;                                  // the minimum value of a channel
// constexpr uint16_t vY_VALUE_MID        = 2100;                                 // the middle value of a channel
// constexpr uint16_t vY_VALUE_MAX        = 3100;                                 // the maximum value of a channel
// constexpr uint16_t vY_LOWER_RESOLUTION = ((vY_VALUE_MID - vY_VALUE_MIN) / 7);  // the absolute range of a channel
// constexpr uint16_t vY_UPPER_RESOLUTION = ((vY_VALUE_MAX - vY_VALUE_MID) / 7);  // the absolute range of a channel

// /**
//  * @brief RC data structure
//  *
//  *         front                             back
//  *    |‾‾‾‾‾‾‾‾‾‾‾‾|                    |‾‾‾‾‾‾‾‾‾‾‾‾|
//  *    |            |                    |            |
//  *    |            |                    |            |
//  *    |            |                    |            |
//  *    |            |                    |            |
//  *    |            |.                  .--.          |
//  *    |            | \                /    \ s7 s6   |
//  *    |            | /                \ en /         |
//  *    |            |'                  '--'          |
//  *    |            |                    |          s3|
//  *    |    .--.    |                    |s2          |
//  *    |   /    \   |                    |            |
//  *    |   \ jy /   |                    |          s4|
//  *    |    '--'    |                    |            |
//  *    |            |                    |            |
//  *    |            |                    |          s5|
//  *    |     s1     |                    |            |
//  *    |            |                    |            |
//  *    |           /                      \           |
//  *    |          /                        \          |
//  *    |         /                          \         |
//  *    |        /                            \        |
//  *    |_______/                              \_______|
//  *
//  */
// class LORAData
// {
//    public:
//     LORAData(uint8_t id) { this->id = id; };
//     LORAData()                            = delete;
//     LORAData(const LORAData &)            = delete;
//     LORAData &operator=(const LORAData &) = delete;

//     /**
//      * @brief init function
//      */
//     void init();

//     /**
//      * @brief Get the RC data
//      */
//     void getLORAData();

//     /**
//      * @brief sent the RC data
//      */
//     void sendLORAData();

//     /**
//      * @brief Tactile Switches, s* = [0, 1]
//      */
//     struct Switches
//     {
//         uint8_t s1 = 0U;
//         uint8_t s2 = 0U;
//         uint8_t s3 = 0U;
//         uint8_t s4 = 0U;
//         uint8_t s5 = 0U;
//         uint8_t s6 = 0U;
//         uint8_t s7 = 0U;
//     };

//     /**
//      * @brief joystick data, vX, vY = [-7, 7], sw = [0, 1]
//      */
//     struct Joystick
//     {
//         int8_t vX      = 0U;
//         int8_t vY      = 0U;
//         uint8_t button = 0U;
//     };

//     /**
//      * @brief encoder data, cnt = [-32768, 32767], dir = [0, 1](1 is po), sw = [0, 1]
//      */
//     struct Encoder
//     {
//         int16_t cnt    = 0U;
//         uint8_t dir    = 1U;
//         uint8_t button = 0U;
//     };

//     /**
//      * @brief connection status
//      */
//     Switches sw;
//     Joystick joystick;
//     Encoder encoder;
//     PacketStatus_t status;
//     RadioOperatingModes_t opMode;
//     int8_t RssiPkt   = -100;
//     bool isConnected = false;
//     uint8_t id;
// };

// /**
//  * @brief put this in HAL_GPIO_EXTI_Callback
//  */
// void gpioEXTICallback(uint16_t GPIO_Pin);
// }  // namespace E282G4M27S
// }  // namespace Drivers
// }  // namespace Core
