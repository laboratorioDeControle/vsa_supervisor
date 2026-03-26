#include <iostream>

#include "../../../include/guidance/control_algorithms/pid.h"
#include "../../../include/guidance/math_operations.h"

using namespace vsa_guidance;

PID::PID(void)
{

}

PID::PID(double kp, double ki, double kd)
{
    initialize(kp, ki, kd);
}

PID::PID(double kp, double ki, double kd, double dt)
{
    initialize(kp, ki, kd, dt);
}

PID::PID(double kp, double ki, double kd, double min_output_value, double max_output_value, double dt)
{
    initialize(kp, ki, kd, min_output_value, max_output_value, dt);
}

PID::PID(guidance_pid_params_t parameters, double dt)
{
    initialize(parameters.kp, parameters.ki, parameters.kd, parameters.min_output, parameters.max_output, dt);
    set_norm_output(parameters.normalize_output);
    set_only_positive_outputs(parameters.only_positives_outputs);
}

PID::~PID()
{

}

void PID::reset(void)
{
    last_error_ = 0.0;
    i_ = 0.0;
}

void PID::set_dt(double dt)
{
    dt_ = dt;
}

void PID::set_setpoint(double setpoint)
{
    setpoint_ = setpoint;
}

void PID::set_kp(double kp)
{
    kp_ = kp;
}

void PID::set_ki(double ki)
{
    ki_ = ki;
}

void PID::set_kd(double kd)
{
    kd_ = kd;
}

void PID::set_max_output_value(double max_output)
{
    max_output_value_ = max_output;
}

void PID::set_min_output_value(double min_output)
{
    min_output_value_ = min_output;
}

void PID::set_saturate_output(bool saturate_output)
{
    saturate_output_ = saturate_output;
}

void PID::set_norm_output(bool norm_output)
{
    norm_output_ = norm_output;
}

void PID::set_only_positive_outputs(bool only_positives_outputs)
{
    only_positives_outputs_ = only_positives_outputs;
}

double PID::get_dt(void)
{
    return dt_;
}

double PID::get_setpoint(void)
{
    return setpoint_;
}

double PID::get_kp(void)
{
    return kp_;
}

double PID::get_ki(void)
{
    return ki_;
}

double PID::get_kd(void)
{
    return kd_;
}

double PID::get_max_output_value(void)
{
    return max_output_value_;
}

double PID::get_min_output_value(void)
{
    return min_output_value_;
}

bool PID::get_saturate_output(void)
{
    return saturate_output_;
}

bool PID::get_norm_output(void)
{
    return norm_output_;
}

bool PID::get_only_positive_outputs(void)
{
    return only_positives_outputs_;
}

void PID::initialize(double kp, double ki, double kd)
{
    set_kp(kp);
    set_ki(ki);
    set_kd(kd);
}
void PID::initialize(double kp, double ki, double kd, double dt)
{
    initialize(kp, ki, kd);
    set_dt(dt);
}

void PID::initialize(double kp, double ki, double kd, double min_output_value, double max_output_value, double dt)
{
    initialize(kp, ki, kd, dt);
    set_min_output_value(min_output_value);
    set_max_output_value(max_output_value);
    set_saturate_output(true);
}

double PID::compute(double input)
{
    if(dt_ == -1)
    {
        return 0.0;
    }

    double error = setpoint_ - input;

    if(wrap_error_)
    {
        double sin_error = std::sin(setpoint_ - input);
        double cos_error = std::cos(setpoint_ - input);

        error = std::atan2(sin_error, cos_error);
    }

    // std::cout << "  error: " << error << std::endl;
    // std::cout << "===================================" << std::endl;

    double p = kp_ * error;

    i_ += ki_ * error * dt_;

    double d = kd_ * ((error - last_error_) / dt_);

    last_error_ = error;

    double output = p + i_ + d;

    //=========================================================================

    if(get_saturate_output())
    {
        output = saturate_output(output);
    }

    if(get_norm_output())
    {
        output = map(output, min_output_value_, max_output_value_, -1, 1);
    }

    if(get_only_positive_outputs())
    {
        if(output < 0.0)
        {
            output = 0.0;
        }
    }

    //=========================================================================

    return output;
}

double PID::compute(double input, double dt)
{
    set_dt(dt);
    return compute(input);
}

double PID::compute(double input, double setpoint, double dt)
{
    set_dt(dt);
    set_setpoint(setpoint);
    return compute(input);
}

double PID::compute(double input, double setpoint, double dt, bool wrap_error)
{
    wrap_error_ = wrap_error;
    set_dt(dt);
    set_setpoint(setpoint);
    return compute(input);
}

double PID::saturate_output(double output)
{
    double result = output;

    if(output > max_output_value_)
    {
        result = max_output_value_;
    }
    else if(output < min_output_value_)
    {
        result = min_output_value_;
    }

    return result;
}