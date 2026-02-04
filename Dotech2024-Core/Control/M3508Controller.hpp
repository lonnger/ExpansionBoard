#pragma once
#include "M3508.hpp"

#if USE_M3508 && defined HAL_CAN_MODULE_ENABLED

#include "FreeRTOS.h"
#include "PID.hpp"
#include "main.h"
#include "task.h"

namespace Core
{
namespace Control
{

struct MotorConfig
{
    float reductionRatio;
    float maxRPM;
    float maxCurrent;
    constexpr MotorConfig(float reductionRatio_, float maxRPM_, float maxCurrent_ = 16383)
        : reductionRatio(reductionRatio_), maxRPM(maxRPM_), maxCurrent(maxCurrent_)
    {
    }
};
enum PIDMode
{
    SINGLE,
    DUOLOOP
};

class M3508Controller
{
   public:
    M3508Controller(MotorConfig &motorconfig_, M3508::M3508 &motor_, const PID<float> &pid_)
        : rpmPID(pid_), positionPID(pid_), motorConfig(motorconfig_), pidMode(PIDMode::SINGLE), motor(motor_)
    {
    }  // for single loop pid, pidParam is used for both rpm and position pid

    M3508Controller(MotorConfig &motorconfig_, M3508::M3508 &motor_, const PID<float> &rpmPID_, const PID<float> &positionPID_)
        : rpmPID(rpmPID_), positionPID(positionPID_), motorConfig(motorconfig_), pidMode(PIDMode::DUOLOOP), motor(motor_)
    {
    }  // for duo loop pid, rpmPIDParam is used for rpm pid, positionPIDParam is used for position pid,
       // and the output of rpm pid is used as the input of position pid

    void setTargetRPM(float targetRPM_);
    void setTargetPosition(float targetPosition_);
    void setRelativePosition(float relativePosition_);  // set the target position as the current position plus the relative position
    void setOutputCurrent(float current);
    void setOffesetPosition(float position);            // set the current position as the offset position
    void update();
    bool isPositionReached();

    float getRPM();
    float getPosition();
    bool isConnected();

   protected:
    PID<float> rpmPID;
    PID<float> positionPID;
    MotorConfig &motorConfig;
    const PIDMode pidMode;
    M3508::M3508 &motor;

   private:
    float targetRPM;       // not the rotor rpm
    float targetPosition;  // not the rotor position
    float relativePosition;
    float offsetPosition;
    float outputCurrent;
    bool connected;
    bool positionReached = 0;
    bool positionLastReached = 0;
    enum ControlMode
    {
        RPM,
        POSITION,
        CURRENT
    };
    ControlMode controlMode = CURRENT;
    bool changeControlMode = false;
    struct feedback
    {
        float rpm;       // reduced rpm output, not the rotor rpm
        float position;  // reduced position output, not the rotor position
    } feedback;
};
}  // namespace Control
}  // namespace Core
#endif