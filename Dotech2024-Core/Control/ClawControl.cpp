#include "ClawControl.hpp"

#if USE_M3508 && defined HAL_CAN_MODULE_ENABLED
namespace ClawControl
{
using namespace Core;
Control::MotorConfig m3508ConfigL{1, 8000, 15000};  //minimum Current/Revolutins Per Minute, maximum Current
static Control::PIDParameters motorPIDLParam{30, 3000, 0, 15000, m3508ConfigL.maxCurrent};
static Control::PIDParameters motorPositionPIDLParam{1000, 0, 0.5, 0, m3508ConfigL.maxRPM};
static Control::PID<float> motorPIDL{motorPIDLParam};
static Control::PID<float> motorPositionPIDL{motorPositionPIDLParam};

static M3508::M3508 &motorL = M3508::getMotor(0x204);
Control::M3508Controller controllerL{m3508ConfigL, motorL, motorPIDL, motorPositionPIDL};

Control::MotorConfig m3508ConfigR{1, 8000, 15000};
static Control::PIDParameters motorPIDRParam{30, 3000, 0, 15000, m3508ConfigR.maxCurrent};
static Control::PIDParameters motorPositionPIDRParam{1000, 0, 0.5, 0, m3508ConfigR.maxRPM};
static Control::PID<float> motorPIDR{motorPIDRParam};
static Control::PID<float> motorPositionPIDR{motorPositionPIDRParam};

extern M3508::M3508 &motorR = M3508::getMotor(0x203);
extern Control::M3508Controller controllerR{m3508ConfigR, motorR, motorPIDR, motorPositionPIDR};

ClawState stateL     = NONE;
ClawState lastStateL = NONE;
ClawCommand commandL = STOP;
ClawState stateR     = NONE;
ClawState lastStateR = NONE;
ClawCommand commandR = STOP;

static uint16_t timeoutL = 0;
static uint16_t waitL    = 0;
static uint16_t timeoutR = 0;
static uint16_t waitR    = 0;

void clawRTask(void *param)
{
    while (1)
    {
        while (commandR != ClawCommand::START)
        {
            vTaskDelay(1);
        }

        stateR              = ClawState::FORWARD;
        m3508ConfigR.maxRPM = 1500;
        timeoutR            = 0;
        controllerR.setRelativePosition(-78.0f);

        uint16_t timeout = 0;

        for (;;)
        {
            if (controllerR.isPositionReached() || timeout++ > 3000 || commandR == ClawCommand::STOP)
            {
                controllerR.setOutputCurrent(0);
                vTaskDelay(40);
                break;
            }
            vTaskDelay(1);
        }

        timeout = 0;

        m3508ConfigR.maxRPM = 5000;
        controllerR.setTargetRPM(3000);
        vTaskDelay(120);

        uint16_t wait = 0;

        stateR = ClawState::BACKWARD;

        for (;;)
        {
            timeout++;
            if (motorR.getActualCurrent() > 9000)
                wait++;
            if (wait > 120 or timeout > 700)
            {
                controllerR.setRelativePosition(-0.5f);
                vTaskDelay(100);
                controllerR.setOutputCurrent(0);
                break;
            }
            vTaskDelay(1);
        }
        stateR   = FINISH;
        commandR = ClawCommand::STOP;
        vTaskDelay(100);
        vTaskDelay(1);
    }
}

ClawState getClawLState() { return stateL; }
ClawState getClawRState() { return stateR; }
bool isClawOK() { return stateL == FINISH && stateR == FINISH; }

void startClaw()
{
    commandL = START;
    commandR = START;
}

void stopClaw()
{
    commandL = STOP;
    commandR = STOP;
}
void stopClawL() { commandL = STOP; }
void stopClawR() { commandR = STOP; }

static StackType_t clawLTaskStack[128];
static StaticTask_t clawLTaskTCB;
static StackType_t clawRTaskStack[128];
static StaticTask_t clawRTaskTCB;
void init() { xTaskCreateStatic(clawRTask, "clawRTask", 128, NULL, 9, clawRTaskStack, &clawRTaskTCB); }
}  // namespace ClawControl

#endif