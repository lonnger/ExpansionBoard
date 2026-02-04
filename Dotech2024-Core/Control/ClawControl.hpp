#pragma once
#include "M3508.hpp"

#if USE_M3508 && defined HAL_CAN_MODULE_ENABLED


#include "FreeRTOS.h"
#include "M3508Controller.hpp"
#include "task.h"

namespace ClawControl
{
enum ClawState
{
    NONE,
    FORWARD,
    BACKWARD,
    FINISH
};
enum ClawCommand
{
    START,
    STOP
};
extern Core::Control::M3508Controller controllerR;
extern Core::Control::M3508Controller controllerL;
void init();
void startClaw();
void stopClaw();
void stopClawL();
void stopClawR();
ClawState getClawLState();
ClawState getClawRState();
bool isClawOK();
}  // namespace ClawControl

#endif