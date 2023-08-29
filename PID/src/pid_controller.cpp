#include "pid_controller.h"

/// @brief
/// @param kp
/// @param ki
/// @param kd
/// @param minOutput
/// @param maxOutput
/// @param sampleTimeSeconds 需要采样时间为s
/// @param mode
/// @param controllerDirection
PIDController::PIDController(const float &kp, const float &ki, const float &kd, const float &minOutput, const float &maxOutput,
                             const float &sampleTimeSeconds, const PIDMode &mode, const PIDDirection &controllerDirection)
{
    controllerDirection_ = controllerDirection;
    mode_ = mode;
    iTerm_ = 0.0f;
    input_ = 0.0f;
    lastInput_ = 0.0f;
    output_ = 0.0f;
    setpoint_ = 0.0f;

    if (sampleTimeSeconds > 0.0f)
    {
        sampleTime_ = sampleTimeSeconds;
    }
    else
    {
        // If the passed parameter was incorrect, set to 1 second
        sampleTime_ = 1.0f;
    }

    PIDOutputLimitsSet(minOutput, maxOutput);
    PIDTuningsSet(kp, ki, kd);
}

PIDController::~PIDController()
{
}

bool PIDController::PIDCompute()
{
    float error, dInput;

    if (mode_ == PIDMode::MANUAL)
    {
        return false;
    }

    // The classic PID error term
    error = setpoint_ - input_;

    // Compute the integral term separately ahead of time
    iTerm_ += alteredKi_ * error;

    // Constrain the integrator to make sure it does not exceed output bounds
    iTerm_ = Sat(iTerm_, outMin_, outMax_);

    // Take the "derivative on measurement" instead of "derivative on error"
    dInput = input_ - lastInput_;

    // Run all the terms together to get the overall output
    output_ = alteredKp_ * error + iTerm_ - alteredKd_ * dInput;

    // Bound the output
    output_ = Sat(output_, outMin_, outMax_);

    // Make the current input the former input
    lastInput_ = input_;

    return true;
}

void PIDController::PIDOutputLimitsSet(const float &min, const float &max)
{
    // Check if the parameters are valid
    if (min >= max)
    {
        return;
    }

    // Save the parameters
    outMin_ = min;
    outMax_ = max;

    // If in automatic, apply the new constraints
    if (mode_ == PIDMode::AUTOMATIC)
    {
        output_ = Sat(output_, min, max);
        iTerm_ = Sat(iTerm_, min, max);
    }
}

void PIDController::PIDTuningsSet(const float &kp, const float &ki, const float &kd)
{
    // Check if the parameters are valid
    if (kp < 0.0f || ki < 0.0f || kd < 0.0f)
    {
        return;
    }

    // Save the parameters for displaying purposes 显示pid
    dispKp_ = kp;
    dispKi_ = ki;
    dispKd_ = kd;

    // Alter the parameters for PID 修改参数的pid
    alteredKp_ = kp;
    alteredKi_ = ki * sampleTime_;
    alteredKd_ = kd / sampleTime_;

    // Apply reverse direction to the altered values if necessary
    if (controllerDirection_ == PIDDirection::REVERSE)
    {
        alteredKp_ = -(alteredKp_);
        alteredKi_ = -(alteredKi_);
        alteredKd_ = -(alteredKd_);
    }
}

void PIDController::PIDModeSet(const PIDMode &mode)
{
    // If the mode changed from MANUAL to AUTOMATIC
    if (mode_ != mode && mode_ == PIDMode::AUTOMATIC)
    {
        // Initialize a few PID parameters to new values
        iTerm_ = output_;
        lastInput_ = input_;

        // Constrain the integrator to make sure it does not exceed output bounds
        iTerm_ = Sat(iTerm_, outMin_, outMax_);
    }

    mode_ = mode;
}

void PIDController::PIDTuningKpSet(const float &kp)
{
    PIDTuningsSet(kp, dispKi_, dispKd_);
}

void PIDController::PIDTuningKiSet(const float &ki)
{
    PIDTuningsSet(dispKp_, ki, dispKd_);
}

void PIDController::PIDTuningKdSet(const float &kd)
{
    PIDTuningsSet(dispKp_, dispKi_, kd);
}

void PIDController::PIDControllerDirectionSet(const PIDDirection &controllerDirection)
{
    // If in automatic mode and the controller's sense of direction is reversed
    if (mode_ == PIDMode::AUTOMATIC && controllerDirection == PIDDirection::REVERSE)
    {
        // Reverse sense of direction of PID gain constants
        alteredKp_ = -(alteredKp_);
        alteredKi_ = -(alteredKi_);
        alteredKd_ = -(alteredKd_);
    }

    controllerDirection_ = controllerDirection;
}

void PIDController::PIDSampleTimeSet(const float &sampleTimeSeconds)
{
    float ratio;

    if (sampleTimeSeconds > 0.0f)
    {
        // Find the ratio of change and apply to the altered values
        ratio = sampleTimeSeconds / sampleTime_;
        alteredKi_ *= ratio;
        alteredKd_ /= ratio;

        // Save the new sampling time
        sampleTime_ = sampleTimeSeconds;
    }
}

float Sat(const float &x, const float &lower, const float &upper)
{
    return (x > upper) ? upper : ((x < lower) ? lower : x);
}
