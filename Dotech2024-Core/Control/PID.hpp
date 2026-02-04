#pragma once

#include "FreeRTOS.h"
#include "task.h"

#define clamp(x, limit) ((x) < (-limit) ? (-limit) : ((x) > (limit) ? (limit) : (x)))
namespace Core
{
namespace Control
{

struct PIDParameters
{
    enum Mode
    {
        POSITION,
        DELTA
    };

    float kp;
    float ki;
    float kd;
    float integralLimit;
    float outputLimit;
    float integralDecay;
    Mode mode;

    constexpr PIDParameters(float kp_ = 0,
                            float ki_ = 0,
                            float kd_ = 0,
                            float integralLimit_ = 30000,  // NO use for POSITION mode
                            float outputLimit_ = 30000,
                            float integralDecay_ = 1,
                            Mode mode_ = Mode::POSITION)  // default is POSITION mode
        : kp(kp_), ki(ki_), kd(kd_), integralLimit(integralLimit_), outputLimit(outputLimit_), integralDecay(integralDecay_), mode(mode_)
    {
    }
};

template <typename T = float>
class PID
{
   public:
    PID(const PIDParameters &parameters_) : parameters(parameters_) {}
    T operator()(T setpoint, T feedback) { return calculate(setpoint, feedback); }

    void reset()
    {
        error[0] = 0;
        error[1] = 0;
        error[2] = 0;
        dBuffer[0] = 0;
        dBuffer[1] = 0;
        dBuffer[2] = 0;
        output = 0;
        pOut = 0;
        iOut = 0;
        dOut = 0;
    }

   private:
    T calculate(T setpoint, T feedback)
    {
        TickType_t t = xTaskGetTickCount();
        TickType_t diff = t - lastTick;
        dt = diff / (float)configTICK_RATE_HZ;
        lastTick = t;
        error[2] = error[1];
        error[1] = error[0];
        error[0] = setpoint - feedback;
        // if (error[0] < 0.1f && error[0] > -0.1f)
        //     error[0] = 0;
        if (parameters.mode == PIDParameters::Mode::POSITION)
        {
            dBuffer[2] = dBuffer[1];
            dBuffer[1] = dBuffer[0];
            dBuffer[0] = (error[0] - error[1]);
            pOut = parameters.kp * error[0];
            dOut = parameters.kd * (error[0] - error[1]) / dt;
            pdOut = pOut + dOut;
            // output     = clamp(pdOut, parameters.outputLimit);
            iOut += parameters.ki * error[0] * dt;

            // iOut = clamp(iOut, parameters.outputLimit - pdOut);
            iOut = clamp(iOut, parameters.integralLimit);
            iOut *= parameters.integralDecay;

            output = iOut + pdOut;
            output = clamp(output, parameters.outputLimit);
        }
        else if (parameters.mode == PIDParameters::Mode::DELTA)
        {
            pOut = parameters.kp * (error[0] - error[1]);
            iOut += parameters.ki * error[0];
            iOut = clamp(iOut, parameters.integralLimit);
            iOut *= parameters.integralDecay;

            dBuffer[2] = dBuffer[1];
            dBuffer[1] = dBuffer[0];
            dBuffer[0] = (0.2f * error[0] - 0.3f * error[1] + 0.5f * error[2]);
            dOut = parameters.kd * dBuffer[0];

            output = pOut + iOut + dOut;
            output = clamp(output, parameters.outputLimit);
        }
        return output;
    }

    const PIDParameters &parameters;
    TickType_t lastTick = 0;
    T error[3] = {0, 0, 0};
    T dBuffer[3] = {0, 0, 0};
    T output = 0;
    T pOut = 0;
    T iOut = 0;
    T dOut = 0;
    T pdOut = 0;
    T dt = 1 / 1000.0f;
};
}  // namespace Control

}  // namespace Core
