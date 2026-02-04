// #include "E282G4M27S.hpp"

// #include <stdio.h>
// #include <string.h>

// #include "main.h"
// #include "radio.h"
// #include "sx1280-hal.h"
// #include "sx1280.h"
// #include "task.h"

// #define MODE_LORA

// #define RF_BL_ADV_CHANNEL_38 2426000000  // Hz
// #define RF_BL_ADV_CHANNEL_0 2404000000   // Hz

// /*!
//  * \brief Defines the nominal frequency
//  */
// #define RF_FREQUENCY RF_BL_ADV_CHANNEL_0  // Hz

// /*!
//  * \brief Defines the output power in dBm
//  *
//  * \remark The range of the output power is [-18..+13] dBm
//  */
// #define TX_OUTPUT_POWER 0

// /*!
//  * \brief Defines the buffer size, i.e. the payload size
//  */
// #define BUFFER_SIZE 4

// /*!
//  * \brief Number of tick size steps for tx timeout
//  */
// #define TX_TIMEOUT_VALUE 100  // ms

// /*!
//  * \brief Number of tick size steps for rx timeout
//  */
// #define RX_TIMEOUT_VALUE 100  // ms

// /*!
//  * \brief Size of ticks (used for Tx and Rx timeout)
//  */
// #define RX_TIMEOUT_TICK_SIZE RADIO_TICK_SIZE_1000_US

// /*!
//  * \brief Defines the size of the token defining message type in the payload
//  */
// #define PINGPONGSIZE 4

// namespace Core
// {
// namespace Drivers
// {
// namespace E282G4M27S
// {
// /*!
//  * \brief Defines the states of the application
//  */
// typedef enum
// {
//     APP_LOWPOWER,
//     APP_RX,
//     APP_RX_TIMEOUT,
//     APP_RX_ERROR,
//     APP_TX,
//     APP_TX_TIMEOUT,
// } AppStates_t;

// /*!
//  * \brief The State of the application
//  */
// AppStates_t AppState    = APP_LOWPOWER;
// AppStates_t SetAppState = APP_LOWPOWER;
// /*!
//  * \brief The size of the buffer
//  */
// static uint8_t BufferSize = BUFFER_SIZE;

// static PacketStatus_t packetStatus;

// /*!
//  * \brief The buffer
//  */
// static uint8_t txBuffer[BUFFER_SIZE];

// static uint8_t rxBuffer[BUFFER_SIZE];

// /*!
//  * \brief Mask of IRQs to listen to in rx mode
//  */
// uint16_t RxIrqMask = IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT;

// /*!
//  * \brief Mask of IRQs to listen to in tx mode
//  */
// uint16_t TxIrqMask = IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT;

// void OnTxDone(void)
// {
//     AppState = APP_TX;
//     if (SetAppState == APP_TX)
//     {
//         Radio.SetDioIrqParams(TxIrqMask, TxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE);
//         Radio.SendPayload(txBuffer, BufferSize, (TickTime_t){RX_TIMEOUT_TICK_SIZE, TX_TIMEOUT_VALUE});
//         HAL_GPIO_WritePin(LORA_RX_EN_GPIO_Port, LORA_RX_EN_Pin, GPIO_PIN_RESET);
//         HAL_GPIO_WritePin(LORA_TX_EN_GPIO_Port, LORA_TX_EN_Pin, GPIO_PIN_SET);
//     }
//     else if (SetAppState == APP_RX)
//     {
//         Radio.SetDioIrqParams(RxIrqMask, RxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE);
//         Radio.SetRx((TickTime_t){RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE});
//         HAL_GPIO_WritePin(LORA_RX_EN_GPIO_Port, LORA_RX_EN_Pin, GPIO_PIN_SET);
//         HAL_GPIO_WritePin(LORA_TX_EN_GPIO_Port, LORA_TX_EN_Pin, GPIO_PIN_RESET);
//     }
// }

// void OnRxDone(void)
// {
//     AppState = APP_RX;
//     if (SetAppState == APP_TX)
//     {
//         Radio.SetDioIrqParams(TxIrqMask, TxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE);
//         Radio.SendPayload(txBuffer, BufferSize, (TickTime_t){RX_TIMEOUT_TICK_SIZE, TX_TIMEOUT_VALUE});
//         HAL_GPIO_WritePin(LORA_RX_EN_GPIO_Port, LORA_RX_EN_Pin, GPIO_PIN_RESET);
//         HAL_GPIO_WritePin(LORA_TX_EN_GPIO_Port, LORA_TX_EN_Pin, GPIO_PIN_SET);
//     }
//     else if (SetAppState == APP_RX)
//     {
//         Radio.GetPayload(rxBuffer, &BufferSize, BUFFER_SIZE);
//         SX1280GetPacketStatus(&packetStatus);
//         Radio.SetDioIrqParams(RxIrqMask, RxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE);
//         Radio.SetRx((TickTime_t){RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE});
//         HAL_GPIO_WritePin(LORA_RX_EN_GPIO_Port, LORA_RX_EN_Pin, GPIO_PIN_SET);
//         HAL_GPIO_WritePin(LORA_TX_EN_GPIO_Port, LORA_TX_EN_Pin, GPIO_PIN_RESET);
//     }
// }

// void OnTxTimeout(void)
// {
//     AppState = APP_TX_TIMEOUT;
//     if (SetAppState == APP_TX)
//     {
//         Radio.SetDioIrqParams(TxIrqMask, TxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE);
//         Radio.SendPayload(txBuffer, BufferSize, (TickTime_t){RX_TIMEOUT_TICK_SIZE, TX_TIMEOUT_VALUE});
//         HAL_GPIO_WritePin(LORA_RX_EN_GPIO_Port, LORA_RX_EN_Pin, GPIO_PIN_RESET);
//         HAL_GPIO_WritePin(LORA_TX_EN_GPIO_Port, LORA_TX_EN_Pin, GPIO_PIN_SET);
//     }
//     else if (SetAppState == APP_RX)
//     {
//         Radio.SetDioIrqParams(RxIrqMask, RxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE);
//         Radio.SetRx((TickTime_t){RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE});
//         HAL_GPIO_WritePin(LORA_RX_EN_GPIO_Port, LORA_RX_EN_Pin, GPIO_PIN_SET);
//         HAL_GPIO_WritePin(LORA_TX_EN_GPIO_Port, LORA_TX_EN_Pin, GPIO_PIN_RESET);
//     }
// }

// void OnRxTimeout(void)
// {
//     AppState = APP_RX_TIMEOUT;
//     if (SetAppState == APP_TX)
//     {
//         Radio.SetDioIrqParams(TxIrqMask, TxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE);
//         Radio.SendPayload(txBuffer, BufferSize, (TickTime_t){RX_TIMEOUT_TICK_SIZE, TX_TIMEOUT_VALUE});
//         HAL_GPIO_WritePin(LORA_RX_EN_GPIO_Port, LORA_RX_EN_Pin, GPIO_PIN_RESET);
//         HAL_GPIO_WritePin(LORA_TX_EN_GPIO_Port, LORA_TX_EN_Pin, GPIO_PIN_SET);
//     }
//     else if (SetAppState == APP_RX)
//     {
//         Radio.SetDioIrqParams(RxIrqMask, RxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE);
//         Radio.SetRx((TickTime_t){RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE});
//         HAL_GPIO_WritePin(LORA_RX_EN_GPIO_Port, LORA_RX_EN_Pin, GPIO_PIN_SET);
//         HAL_GPIO_WritePin(LORA_TX_EN_GPIO_Port, LORA_TX_EN_Pin, GPIO_PIN_RESET);
//     }
// }

// void OnRxError(IrqErrorCode_t errorCode)
// {
//     AppState = APP_RX_ERROR;
//     if (SetAppState == APP_TX)
//     {
//         Radio.SetDioIrqParams(TxIrqMask, TxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE);
//         Radio.SendPayload(txBuffer, BufferSize, (TickTime_t){RX_TIMEOUT_TICK_SIZE, TX_TIMEOUT_VALUE});
//         HAL_GPIO_WritePin(LORA_RX_EN_GPIO_Port, LORA_RX_EN_Pin, GPIO_PIN_RESET);
//         HAL_GPIO_WritePin(LORA_TX_EN_GPIO_Port, LORA_TX_EN_Pin, GPIO_PIN_SET);
//     }
//     else if (SetAppState == APP_RX)
//     {
//         Radio.SetDioIrqParams(RxIrqMask, RxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE);
//         Radio.SetRx((TickTime_t){RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE});
//         HAL_GPIO_WritePin(LORA_RX_EN_GPIO_Port, LORA_RX_EN_Pin, GPIO_PIN_SET);
//         HAL_GPIO_WritePin(LORA_TX_EN_GPIO_Port, LORA_TX_EN_Pin, GPIO_PIN_RESET);
//     }
// }

// void OnRangingDone(IrqRangingCode_t val) {}

// void OnCadDone(bool channelActivityDetected) {}

// /*!
//  * \brief All the callbacks are stored in a structure
//  */
// RadioCallbacks_t Callbacks = {
//     &OnTxDone,     // txDone
//     &OnRxDone,     // rxDone
//     NULL,          // syncWordDone
//     NULL,          // headerDone
//     &OnTxTimeout,  // txTimeout
//     &OnRxTimeout,  // rxTimeout
//     &OnRxError,    // rxError
//     NULL,          // rangingDone
//     NULL,          // cadDone
// };

// PacketParams_t packetParams;

// void LORAData::sendLORAData()
// {
//     char output[BUFFER_SIZE + 1];
//     uint8_t swByte = ((this->joystick.button & 1) << 7) | ((this->encoder.button & 1) << 6) | ((this->sw.s7 & 1) << 5) | ((this->sw.s6 & 1) << 4) |
//                      ((this->sw.s5 & 1) << 3) | ((this->sw.s4 & 1) << 2) | ((this->sw.s3 & 1) << 1) | ((this->sw.s2 & 1) << 0);

//     int8_t joystickByte = ((this->joystick.vY & 0x0F) << 4) | (this->joystick.vX & 0x0F);

//     uint8_t encoderCntHighByte = (this->encoder.cnt >> 8) & 0xFF;
//     uint8_t encoderCntLowByte  = this->encoder.cnt & 0xFF;

//     output[0] = swByte;
//     output[1] = joystickByte;
//     output[2] = encoderCntHighByte;
//     output[3] = encoderCntLowByte;
//     memcpy(txBuffer, output, BUFFER_SIZE);
//     SetAppState = APP_TX;
//     if (AppState == APP_LOWPOWER)
//     {
//         Radio.SetDioIrqParams(TxIrqMask, TxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE);
//         Radio.SendPayload(txBuffer, BufferSize, (TickTime_t){RX_TIMEOUT_TICK_SIZE, TX_TIMEOUT_VALUE});
//         HAL_GPIO_WritePin(LORA_RX_EN_GPIO_Port, LORA_RX_EN_Pin, GPIO_PIN_RESET);
//         HAL_GPIO_WritePin(LORA_TX_EN_GPIO_Port, LORA_TX_EN_Pin, GPIO_PIN_SET);
//     }
// }

// void LORAData::getLORAData()
// {
//     SetAppState = APP_RX;
//     if (AppState == APP_LOWPOWER)
//     {
//         Radio.SetDioIrqParams(RxIrqMask, RxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE);
//         Radio.SetRx((TickTime_t){RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE});
//         HAL_GPIO_WritePin(LORA_RX_EN_GPIO_Port, LORA_RX_EN_Pin, GPIO_PIN_SET);
//         HAL_GPIO_WritePin(LORA_TX_EN_GPIO_Port, LORA_TX_EN_Pin, GPIO_PIN_RESET);
//     }
//     char output[BUFFER_SIZE + 1];
//     memcpy(output, rxBuffer, BUFFER_SIZE);
//     uint8_t swByte             = output[0];
//     uint8_t joystickByte       = output[1];
//     uint8_t encoderCntHighByte = output[2];
//     uint8_t encoderCntLowByte  = output[3];

//     this->sw.s2           = (swByte & 0x01) != 0;
//     this->sw.s3           = (swByte & 0x02) != 0;
//     this->sw.s4           = (swByte & 0x04) != 0;
//     this->sw.s5           = (swByte & 0x08) != 0;
//     this->sw.s6           = (swByte & 0x10) != 0;
//     this->sw.s7           = (swByte & 0x20) != 0;
//     this->encoder.button  = (swByte & 0x40) != 0;
//     this->joystick.button = (swByte & 0x80) != 0;

//     this->joystick.vX = (joystickByte & 0x08) == 0x08 ? (joystickByte & 0x0F) | 0xF0 : joystickByte & 0x0F;
//     this->joystick.vY = ((joystickByte >> 4) & 0x08) == 0x08 ? (joystickByte >> 4) & 0x0F | 0xF0 : (joystickByte >> 4) & 0x0F;

//     this->encoder.cnt = (encoderCntHighByte << 8) | encoderCntLowByte;
// }

// void LORAData::init()
// {
//     HAL_GPIO_WritePin(LORA_TX_EN_GPIO_Port, LORA_TX_EN_Pin, GPIO_PIN_RESET);
//     HAL_GPIO_WritePin(LORA_RX_EN_GPIO_Port, LORA_RX_EN_Pin, GPIO_PIN_RESET);

//     ModulationParams_t modulationParams;

//     Radio.Init(&Callbacks);

//     // vTaskDelay(500);
//     Radio.SetRegulatorMode(USE_DCDC);  // Can also be set in LDO mode but consume more power
//     memset(&rxBuffer, 0x00, BufferSize);
//     memset(&txBuffer, 0x00, BufferSize);

//     modulationParams.PacketType                  = PACKET_TYPE_LORA;
//     modulationParams.Params.LoRa.SpreadingFactor = LORA_SF8;
//     modulationParams.Params.LoRa.Bandwidth       = LORA_BW_1600;
//     modulationParams.Params.LoRa.CodingRate      = LORA_CR_LI_4_5;

//     packetParams.PacketType                 = PACKET_TYPE_LORA;
//     packetParams.Params.LoRa.PreambleLength = 12;
//     packetParams.Params.LoRa.HeaderType     = LORA_PACKET_VARIABLE_LENGTH;
//     packetParams.Params.LoRa.PayloadLength  = BUFFER_SIZE;
//     packetParams.Params.LoRa.CrcMode        = LORA_CRC_ON;
//     packetParams.Params.LoRa.InvertIQ       = LORA_IQ_NORMAL;

//     Radio.SetStandby(STDBY_RC);
//     Radio.SetPacketType(modulationParams.PacketType);
//     Radio.SetModulationParams(&modulationParams);
//     Radio.SetPacketParams(&packetParams);
//     Radio.SetRfFrequency(RF_FREQUENCY);
//     Radio.SetBufferBaseAddresses(0x00, 0x00);
//     Radio.SetTxParams(TX_OUTPUT_POWER, RADIO_RAMP_02_US);

//     Radio.SetInterruptMode();

//     AppState    = APP_LOWPOWER;
//     SetAppState = APP_LOWPOWER;
// }

// void gpioEXTICallback(uint16_t GPIO_Pin) { SX1280HalGpioCallback(GPIO_Pin); }

// }  // namespace E282G4M27S
// }  // namespace Drivers
// }  // namespace Core