#pragma once
#include "appConfig.hpp"

#ifndef USE_CAN
#define USE_CAN 0
#endif


#if USE_CAN
// #include "fdcan.h"
#include "main.h"
#include "stdint.h"

#ifdef HAL_CAN_MODULE_ENABLED
#include "can.h"
namespace CAN
{
void init();
void callback(CAN_HandleTypeDef *, uint32_t);

}  // namespace FDCAN

#elif defined HAL_FDCAN_MODULE_ENABLED
#include "fdcan.h"
namespace FDCAN
{
void init();
void callback(FDCAN_HandleTypeDef *, uint32_t);

}  // namespace FDCAN

#endif
#endif