#pragma once
#include "appConfig.hpp"
#ifndef USE_DYPA21_PWM
#define USE_DYPA21_PWM 1
#endif

#if USE_DYPA21_PWM

#include "tim.h"
namespace DYPA21_PWM
{
class DYPA21_PWM
{
   public:
    DYPA21_PWM(TIM_HandleTypeDef *tim_handle);

    int16_t readData();

   private:
    TIM_HandleTypeDef *tim_handle;
};
}  // namespace DYPA21_PWM
#endif