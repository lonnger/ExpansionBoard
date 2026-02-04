#include "M3508Controller.hpp"

#if USE_M3508 && defined HAL_CAN_MODULE_ENABLED

#include "math.h"
namespace Core
{
namespace Control
{
#define ABS(x) (((x) > 0) ? (x) : (-(x)))

void M3508Controller::setOutputCurrent(float current)
{
    this->outputCurrent = current;
    if (this->controlMode != ControlMode::CURRENT)
        changeControlMode = true;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
    this->controlMode = ControlMode::CURRENT;
}
void M3508Controller::setOffesetPosition(float position) { motor.setPosition(position * motorConfig.reductionRatio); }
void M3508Controller::setTargetRPM(float targetRPM_)
{
    this->targetRPM = targetRPM_;
    if (this->controlMode != ControlMode::RPM)
        changeControlMode = true;
    this->controlMode = ControlMode::RPM;
}

void M3508Controller::setTargetPosition(float targetPosition_)
{
    this->targetPosition = targetPosition_;
    if (this->controlMode != ControlMode::POSITION)
        changeControlMode = true;
    this->controlMode = ControlMode::POSITION;
}

float M3508Controller::getRPM() { return (float)(motor.getRPM()) / motorConfig.reductionRatio; }

float M3508Controller::getPosition() { return (float)(motor.getPosition()) / motorConfig.reductionRatio; }

bool M3508Controller::isConnected() { return connected; }
bool M3508Controller::isPositionReached() { return positionReached; }

void M3508Controller::setRelativePosition(float relativePosition_)
{

    this->relativePosition = relativePosition_;

    // if (positionReached && !positionLastReached)
    //     this->relativePosition = 0;
    // if (positionReached && relativePosition != 0)
        // this->targetPosition += this->relativePosition;
    this->targetPosition = motor.getPosition() / motorConfig.reductionRatio + this->relativePosition;

    if (relativePosition != 0)
    {
        this->positionReached = false;
    }

    if (this->controlMode != ControlMode::POSITION)
        changeControlMode = true;
    this->controlMode = ControlMode::POSITION;
}

void M3508Controller::update()
{
    this->connected = motor.isConnected();
    this->feedback.rpm = getRPM();
    this->feedback.position = getPosition();
    this->positionLastReached = this->positionReached;

    // if (ABS(this->feedback.position - this->targetPosition) < 0.1f)
    //     this->positionReached = true;
    // else
    //     this->positionReached = false;
    this->positionReached = ABS(this->feedback.position - this->targetPosition) < 0.5f;

    float targetRotorRPM = targetRPM * motorConfig.reductionRatio;
    float currentRotorRPM = motor.getRPM();
    float targetRotorPosition = targetPosition * motorConfig.reductionRatio;
    float currentRotorPosition = motor.getPosition();
    if (pidMode == PIDMode::SINGLE)
    {
        if (controlMode == ControlMode::RPM)
        {
            this->outputCurrent = rpmPID(targetRotorRPM, currentRotorRPM);
        }
        else if (controlMode == ControlMode::POSITION)
        {
            this->outputCurrent = positionPID(targetRotorPosition, currentRotorPosition);
        }
        motor.setOutputCurrent(this->outputCurrent);
    }
    else if (pidMode == PIDMode::DUOLOOP)
    {
        switch (controlMode)
        {
        case ControlMode::POSITION:
        {
            float rpmCal = positionPID(targetRotorPosition, currentRotorPosition);
            if (rpmCal > motorConfig.maxRPM)
                rpmCal = motorConfig.maxRPM;
            else if (rpmCal < -motorConfig.maxRPM)
                rpmCal = -motorConfig.maxRPM;
            outputCurrent = rpmPID(rpmCal, currentRotorRPM);
            break;
        }
        case ControlMode::RPM:
        {
            outputCurrent = rpmPID(targetRotorRPM, currentRotorRPM);
            break;
        }
            // FALLTHROUGH
        default:
            break;
        }
        motor.setOutputCurrent(outputCurrent);
    }

    if (!connected || changeControlMode)
    {
        changeControlMode = false;
        rpmPID.reset();
        positionPID.reset();
    }
}

}  // namespace Control
}  // namespace Core
#endif