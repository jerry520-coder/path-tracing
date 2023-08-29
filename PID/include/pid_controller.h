#ifndef PID_CONTROLLER_h
#define PID_CONTROLLER_h

enum class PIDMode : char
{
    MANUAL,
    AUTOMATIC
};

enum class PIDDirection : char
{
    DIRECT,
    REVERSE
};

class PIDController
{
public:
    PIDController(const float &kp, const float &ki, const float &kd, const float &minOutput, const float &maxOutput,
                  const float &sampleTimeSeconds, const PIDMode &mode, const PIDDirection &controllerDirection);
    ~PIDController();

    bool PIDCompute();

    void PIDModeSet(const PIDMode &mode);
    void PIDOutputLimitsSet(const float &min, const float &max);

    void PIDTuningsSet(const float &kp, const float &ki, const float &kd);
    void PIDTuningKpSet(const float &kp);
    void PIDTuningKiSet(const float &ki);
    void PIDTuningKdSet(const float &kd);

    void PIDControllerDirectionSet(const PIDDirection &controllerDirection);
    void PIDSampleTimeSet(const float &sampleTimeSeconds);

    inline void PIDSetpointSet(const float &setpoint) { this->setpoint_ = setpoint; }
    inline void PIDInputSet(const float &input) { this->input_ = input; }
    inline float PIDOutputGet() { return this->output_; }
    inline float PIDKpGet() { return this->dispKp_; }
    inline float PIDKiGet() { return this->dispKi_; }
    inline float PIDKdGet() { return this->dispKd_; }
    inline PIDMode PIDModeGet() { return this->mode_; }
    inline PIDDirection PIDDirectionGet() { return this->controllerDirection_; }

private:
    //
    // Input to the PID Controller
    //
    float input_;

    //
    // Previous input to the PID Controller
    //
    float lastInput_;

    //
    // Output of the PID Controller
    //
    float output_;

    //
    // Gain constant values that were passed by the user
    // These are for display purposes 显示pid
    //
    float dispKp_;
    float dispKi_;
    float dispKd_;

    //
    // Gain constant values that the controller alters for
    // its own use 修改参数的pid
    //
    float alteredKp_;
    float alteredKi_;
    float alteredKd_;

    //
    // The Integral Term
    //
    float iTerm_;

    //
    // The interval (in seconds) on which the PID controller
    // will be called
    //
    float sampleTime_;

    //
    // The values that the output will be constrained to
    //
    float outMin_;
    float outMax_;

    //
    // The user chosen operating point
    //
    float setpoint_;

    //
    // The sense of direction of the controller
    // DIRECT:  A positive setpoint gives a positive output
    // REVERSE: A positive setpoint gives a negative output
    //
    PIDDirection controllerDirection_;

    //
    // Tells how the controller should respond if the user has
    // taken over manual control or not
    // MANUAL:    PID controller is off.
    // AUTOMATIC: PID controller is on.
    //
    PIDMode mode_;
};

float Sat(const float &x, const float &lower, const float &upper);

#endif