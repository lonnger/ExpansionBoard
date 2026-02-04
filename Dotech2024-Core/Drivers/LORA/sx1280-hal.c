// /*
//   ______                              _
//  / _____)             _              | |
// ( (____  _____ ____ _| |_ _____  ____| |__
//  \____ \| ___ |    (_   _) ___ |/ ___)  _ \
//  _____) ) ____| | | || |_| ____( (___| | | |
// (______/|_____)_|_|_| \__)_____)\____)_| |_|
//     (C)2016 Semtech

// Description: Handling of the node configuration protocol

// License: Revised BSD License, see LICENSE.TXT file include in the project

// Maintainer: Miguel Luis, Matthieu Verdy and Benjamin Boulet
// */
// #include "sx1280-hal.h"
// #include "radio.h"
// #include <string.h>
// #include "main.h"
// #include "spi.h"

// /*!
//  * \brief Define the size of tx and rx hal buffers
//  *
//  * The Tx and Rx hal buffers are used for SPI communication to
//  * store data to be sent/receive to/from the chip.
//  *
//  * \warning The application must ensure the maximal useful size to be much lower
//  *          than the MAX_HAL_BUFFER_SIZE
//  */
// #define MAX_HAL_BUFFER_SIZE   0xFFF

// #define IRQ_HIGH_PRIORITY  0

// /*!
//  * Radio driver structure initialization
//  */
// const struct Radio_s Radio =
// {
//     SX1280Init,
//     SX1280HalReset,
//     SX1280GetStatus,
//     SX1280HalWriteCommand,
//     SX1280HalReadCommand,
//     SX1280HalWriteRegisters,
//     SX1280HalWriteRegister,
//     SX1280HalReadRegisters,
//     SX1280HalReadRegister,
//     SX1280HalWriteBuffer,
//     SX1280HalReadBuffer,
//     SX1280HalGetDioStatus,
//     SX1280GetFirmwareVersion,
//     SX1280SetRegulatorMode,
//     SX1280SetStandby,
//     SX1280SetPacketType,
//     SX1280SetModulationParams,
//     SX1280SetPacketParams,
//     SX1280SetRfFrequency,
//     SX1280SetBufferBaseAddresses,
//     SX1280SetTxParams,
//     SX1280SetDioIrqParams,
//     SX1280SetSyncWord,
//     SX1280SetRx,
//     SX1280GetPayload,
//     SX1280SendPayload,
//     SX1280SetRangingRole,
//     SX1280SetPollingMode,
//     SX1280SetInterruptMode,
//     SX1280SetRegistersDefault,
//     SX1280GetOpMode,
//     SX1280SetSleep,
//     SX1280SetFs,
//     SX1280SetTx,
//     SX1280SetRxDutyCycle,
//     SX1280SetCad,
//     SX1280SetTxContinuousWave,
//     SX1280SetTxContinuousPreamble,
//     SX1280GetPacketType,
//     SX1280SetCadParams,
//     SX1280GetRxBufferStatus,
//     SX1280GetPacketStatus,
//     SX1280GetRssiInst,
//     SX1280GetIrqStatus,
//     SX1280ClearIrqStatus,
//     SX1280Calibrate,
//     SX1280SetSaveContext,
//     SX1280SetAutoTx,
//     SX1280StopAutoTx,
//     SX1280SetAutoFS,
//     SX1280SetLongPreamble,
//     SX1280SetPayload,
//     SX1280SetSyncWordErrorTolerance,
//     SX1280SetCrcSeed,
//     SX1280SetBleAccessAddress,
//     SX1280SetBleAdvertizerAccessAddress,
//     SX1280SetCrcPolynomial,
//     SX1280SetWhiteningSeed,
//     SX1280EnableManualGain,
//     SX1280DisableManualGain,
//     SX1280SetManualGainValue,
//     SX1280SetLNAGainSetting,
//     SX1280SetRangingIdLength,
//     SX1280SetDeviceRangingAddress,
//     SX1280SetRangingRequestAddress,
//     SX1280GetRangingResult,
//     SX1280SetRangingCalibration,
//     SX1280GetRangingPowerDeltaThresholdIndicator,
//     SX1280RangingClearFilterResult,
//     SX1280RangingSetFilterNumSamples,
//     SX1280GetFrequencyError,
// };

// static uint8_t halTxBuffer[MAX_HAL_BUFFER_SIZE] = {0x00};
// static uint8_t halRxBuffer[MAX_HAL_BUFFER_SIZE] = {0x00};

// /*!
//  * \brief Used to block execution waiting for low state on radio busy pin.
//  *        Essentially used in SPI communications
//  */
// void SX1280HalWaitOnBusy( void )
// {
//     while( HAL_GPIO_ReadPin( LORABUSY_GPIO_Port, LORABUSY_Pin ) == 1 );
// }

// typedef void( GpioIrqHandler )( void );
// static GpioIrqHandler *gpioIrq[16] = { NULL };
// uint8_t GpioGetBitPos( uint16_t GPIO_Pin )
// {
//     uint8_t pinPos = 0u;

//     if ( ( GPIO_Pin & 0xFF00u ) != 0u ){
//         pinPos |= 0x8u;
//     }
//     if ( ( GPIO_Pin & 0xF0F0u ) != 0u ){
//         pinPos |= 0x4u;
//     }
//     if ( ( GPIO_Pin & 0xCCCCu ) != 0u ){
//         pinPos |= 0x2u;
//     }
//     if ( ( GPIO_Pin & 0xAAAAu ) != 0u ){
//         pinPos |= 0x1u;
//     }

//     return pinPos;
// }
// void GpioSetIrq_modified( uint16_t GPIO_Pin, GpioIrqHandler *irqHandler )
// {
//     uint32_t    BitPos = GpioGetBitPos( GPIO_Pin ) ;

//     if ( irqHandler != NULL )
//     {
// 		gpioIrq[BitPos] = irqHandler;
//     }
// }
// void SX1280HalInit( DioIrqHandler **irqHandlers)
// {
//     SX1280HalReset( );
//     // SX1280HalIoIrqInit( irqHandlers );
//     GpioSetIrq_modified(LORADIO1_Pin, irqHandlers[0]);
// }
// IRQn_Type MSP_GetIRQn( uint16_t GPIO_Pin )
// {
//   switch( GPIO_Pin )
//   {
//     case GPIO_PIN_0:  return EXTI0_IRQn;
//     case GPIO_PIN_1:  return EXTI1_IRQn;
//     case GPIO_PIN_2:  return EXTI2_IRQn;
//     case GPIO_PIN_3:  return EXTI3_IRQn;
//     case GPIO_PIN_4:  return EXTI4_IRQn;
//     case GPIO_PIN_5:  
//     case GPIO_PIN_6:
//     case GPIO_PIN_7:
//     case GPIO_PIN_8:
//     case GPIO_PIN_9:  return EXTI9_5_IRQn;
//     case GPIO_PIN_10:
//     case GPIO_PIN_11:
//     case GPIO_PIN_12:
//     case GPIO_PIN_13:
//     case GPIO_PIN_14:
//     case GPIO_PIN_15: 
//     default: return EXTI15_10_IRQn;
//   }
// }
// void GpioSetIrq( GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t prio,  GpioIrqHandler *irqHandler )
// {
//     IRQn_Type   IRQnb;
//     uint32_t    BitPos = GpioGetBitPos( GPIO_Pin ) ;

//     if ( irqHandler != NULL )
//     {
// 		gpioIrq[BitPos] = irqHandler;

//         IRQnb = MSP_GetIRQn( GPIO_Pin );

//         HAL_NVIC_SetPriority( IRQnb , prio, 0u );

//         HAL_NVIC_EnableIRQ( IRQnb );
//     }
// }
// void SX1280HalGpioCallback( uint16_t GPIO_Pin )
// {
//     uint32_t BitPos = GpioGetBitPos( GPIO_Pin );

//     if ( gpioIrq[BitPos]  != NULL )
//     {
//         gpioIrq[BitPos]( );
//     }
// }

// // void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin )
// // {
// //   GpioLaunchIrqHandler( GPIO_Pin );
// // }
// void SX1280HalIoIrqInit( DioIrqHandler **irqHandlers )
// {
// #if( RADIO_DIO1_ENABLE )
//     GpioSetIrq( LORADIO1_GPIO_Port, LORADIO1_Pin, IRQ_HIGH_PRIORITY, irqHandlers[0] );
// #endif
// #if( RADIO_DIO2_ENABLE )
// 	GpioSetIrq( RADIO_DIO2_GPIO_Port, RADIO_DIO2_Pin, IRQ_HIGH_PRIORITY, irqHandlers[0] );
// #endif
// #if( RADIO_DIO3_ENABLE )
// 	GpioSetIrq( RADIO_DIO3_GPIO_Port, RADIO_DIO3_Pin, IRQ_HIGH_PRIORITY, irqHandlers[0] );
// #endif
// #if( !RADIO_DIO1_ENABLE && !RADIO_DIO2_ENABLE && !RADIO_DIO3_ENABLE )
// #error "Please define a DIO" 
// #endif
// }


// void SX1280HalReset( void )
// {
//     vTaskDelay( 20 );
//     HAL_GPIO_WritePin( LORANRESET_GPIO_Port, LORANRESET_Pin, 0 );
//     vTaskDelay( 50 );
//     HAL_GPIO_WritePin( LORANRESET_GPIO_Port, LORANRESET_Pin, 1 );
//     vTaskDelay( 20 );
// }

// void SX1280HalClearInstructionRam( void )
// {
//     // Clearing the instruction RAM is writing 0x00s on every bytes of the
//     // instruction RAM
//     uint16_t halSize = 3 + IRAM_SIZE;
//     halTxBuffer[0] = RADIO_WRITE_REGISTER;
//     halTxBuffer[1] = ( IRAM_START_ADDRESS >> 8 ) & 0x00FF;
//     halTxBuffer[2] = IRAM_START_ADDRESS & 0x00FF;
//     for( uint16_t index = 0; index < IRAM_SIZE; index++ )
//     {
//         halTxBuffer[3+index] = 0x00;
//     }

//     SX1280HalWaitOnBusy( );

//     HAL_GPIO_WritePin( LORANSS_GPIO_Port, LORANSS_Pin, 0 );

//     HAL_SPI_Transmit( &LORA_SPI, halTxBuffer, halSize , HAL_MAX_DELAY);

//     HAL_GPIO_WritePin( LORANSS_GPIO_Port, LORANSS_Pin, 1 );

//     SX1280HalWaitOnBusy( );
// }

// void SX1280HalWakeup( void )
// {
//     // __disable_irq( );

//     HAL_GPIO_WritePin( LORANSS_GPIO_Port, LORANSS_Pin, 0 );

//     uint16_t halSize = 2;
//     halTxBuffer[0] = RADIO_GET_STATUS;
//     halTxBuffer[1] = 0x00;
//     HAL_SPI_Transmit( &LORA_SPI, halTxBuffer, halSize , HAL_MAX_DELAY);

//     HAL_GPIO_WritePin( LORANSS_GPIO_Port, LORANSS_Pin, 1 );

//     // Wait for chip to be ready.
//     SX1280HalWaitOnBusy( );

//     // __enable_irq( );
// }

// void SX1280HalWriteCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
// {
//     uint16_t halSize  = size + 1;
//     SX1280HalWaitOnBusy( );

//     HAL_GPIO_WritePin( LORANSS_GPIO_Port, LORANSS_Pin, 0 );

//     halTxBuffer[0] = command;

//     memcpy( halTxBuffer + 1, ( uint8_t * )buffer, size * sizeof( uint8_t ) );

// static volatile HAL_StatusTypeDef state;
//     state = HAL_SPI_Transmit( &LORA_SPI, halTxBuffer, halSize , HAL_MAX_DELAY);

//     // HAL_Delay(1);

//     HAL_GPIO_WritePin( LORANSS_GPIO_Port, LORANSS_Pin, 1 );

//     if( command != RADIO_SET_SLEEP ) 
//     {
//         SX1280HalWaitOnBusy( );
//     }
// }

// void SX1280HalReadCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
// {
//     uint16_t halSize = 2 + size;
//     halTxBuffer[0] = command;
//     halTxBuffer[1] = 0x00;
//     for( uint16_t index = 0; index < size; index++ )
//     {
//         halTxBuffer[2+index] = 0x00;
//     }

//     SX1280HalWaitOnBusy( );

//     HAL_GPIO_WritePin( LORANSS_GPIO_Port, LORANSS_Pin, 0 );

//     HAL_SPI_TransmitReceive( &LORA_SPI, halTxBuffer, halRxBuffer, halSize , HAL_MAX_DELAY);

//     memcpy( buffer, halRxBuffer + 2, size );

//     HAL_GPIO_WritePin( LORANSS_GPIO_Port, LORANSS_Pin, 1 );

//     SX1280HalWaitOnBusy( );
// }

// void SX1280HalWriteRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
// {
//     uint16_t halSize = size + 3;
//     halTxBuffer[0] = RADIO_WRITE_REGISTER;
//     halTxBuffer[1] = ( address & 0xFF00 ) >> 8;
//     halTxBuffer[2] = address & 0x00FF;
//     memcpy( halTxBuffer + 3, buffer, size );

//     SX1280HalWaitOnBusy( );

//     HAL_GPIO_WritePin( LORANSS_GPIO_Port, LORANSS_Pin, 0 );

//     HAL_SPI_Transmit( &LORA_SPI, halTxBuffer, halSize , HAL_MAX_DELAY);

//     HAL_GPIO_WritePin( LORANSS_GPIO_Port, LORANSS_Pin, 1 );

//     SX1280HalWaitOnBusy( );
// }

// void SX1280HalWriteRegister( uint16_t address, uint8_t value )
// {
//     SX1280HalWriteRegisters( address, &value, 1 );
// }

// void SX1280HalReadRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
// {
//     uint16_t halSize = 4 + size;
//     halTxBuffer[0] = RADIO_READ_REGISTER;
//     halTxBuffer[1] = ( address & 0xFF00 ) >> 8;
//     halTxBuffer[2] = address & 0x00FF;
//     halTxBuffer[3] = 0x00;
//     for( uint16_t index = 0; index < size; index++ )
//     {
//         halTxBuffer[4+index] = 0x00;
//     }

//     SX1280HalWaitOnBusy( );

//     HAL_GPIO_WritePin( LORANSS_GPIO_Port, LORANSS_Pin, 0 );

//     HAL_SPI_TransmitReceive( &LORA_SPI, halTxBuffer, halRxBuffer, halSize , HAL_MAX_DELAY);

//     memcpy( buffer, halRxBuffer + 4, size );

//     HAL_GPIO_WritePin( LORANSS_GPIO_Port, LORANSS_Pin, 1 );

//     SX1280HalWaitOnBusy( );
// }

// uint8_t SX1280HalReadRegister( uint16_t address )
// {
//     uint8_t data;

//     SX1280HalReadRegisters( address, &data, 1 );

//     return data;
// }

// void SX1280HalWriteBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
// {
//     uint16_t halSize = size + 2;
//     halTxBuffer[0] = RADIO_WRITE_BUFFER;
//     halTxBuffer[1] = offset;
//     memcpy( halTxBuffer + 2, buffer, size );

//     SX1280HalWaitOnBusy( );

//     HAL_GPIO_WritePin( LORANSS_GPIO_Port, LORANSS_Pin, 0 );

//     HAL_SPI_Transmit( &LORA_SPI, halTxBuffer, halSize , HAL_MAX_DELAY);

//     HAL_GPIO_WritePin( LORANSS_GPIO_Port, LORANSS_Pin, 1 );

//     SX1280HalWaitOnBusy( );
// }

// void SX1280HalReadBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
// {
//     uint16_t halSize = size + 3;
//     halTxBuffer[0] = RADIO_READ_BUFFER;
//     halTxBuffer[1] = offset;
//     halTxBuffer[2] = 0x00;
//     for( uint16_t index = 0; index < size; index++ )
//     {
//         halTxBuffer[3+index] = 0x00;
//     }

//     SX1280HalWaitOnBusy( );

//     HAL_GPIO_WritePin( LORANSS_GPIO_Port, LORANSS_Pin, 0 );

//     HAL_SPI_TransmitReceive( &LORA_SPI, halTxBuffer, halRxBuffer, halSize , HAL_MAX_DELAY);

//     memcpy( buffer, halRxBuffer + 3, size );

//     HAL_GPIO_WritePin( LORANSS_GPIO_Port, LORANSS_Pin, 1 );

//     SX1280HalWaitOnBusy( );
// }

// uint8_t SX1280HalGetDioStatus( void )
// {
// 	uint8_t Status = HAL_GPIO_ReadPin( LORABUSY_GPIO_Port, LORABUSY_Pin );
	
// #if( RADIO_DIO1_ENABLE )
// 	Status |= (HAL_GPIO_ReadPin( LORADIO1_GPIO_Port, LORADIO1_Pin ) << 1);
// #endif
// #if( RADIO_DIO2_ENABLE )
// 	Status |= (GpioRead( RADIO_DIO2_GPIO_Port, RADIO_DIO2_Pin ) << 2);
// #endif
// #if( RADIO_DIO3_ENABLE )
// 	Status |= (GpioRead( RADIO_DIO3_GPIO_Port, RADIO_DIO3_Pin ) << 3);
// #endif
// #if( !RADIO_DIO1_ENABLE && !RADIO_DIO2_ENABLE && !RADIO_DIO3_ENABLE )
// #error "Please define a DIO" 
// #endif
	
// 	return Status;
// }
