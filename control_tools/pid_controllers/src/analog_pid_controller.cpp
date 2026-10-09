// Copyright (c) 2024, ISI SIM
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include <cmath>
#include <iostream>

#include "pid_controllers/analog_pid_controller.hpp"

namespace PIDControllers
{

    AnalogPID::AnalogPID(const double kp,
                         const double ki,
                         const double kd,
                         const double umin,
                         const double umax,
                         const double tau,
                         std::optional<std::string> info) : control_output_raw_(0.0),
                                                            control_saturated_(false)
    {
        set_info(info.value_or(""));
        set_gain_kp(kp);
        set_gain_ki(ki);
        set_gain_kd(kd);
        set_output_saturation(umin, umax);
        set_filter_coeficient(tau);

        reset();
    }

    double AnalogPID::compute(const double setpoint, const double measured_value, const double dt)
    {
        double error_ = setpoint - measured_value;

        if (std::isnan(error_))
        {
            if (!info_.empty())
            {
                std::cout << " * " << info_.c_str();
                error_ = 0;
            }
            std::cout << " error is NaN" << std::endl;
        }

        if (std::isinf(error_))
        {
            if (!info_.empty())
            {
                std::cout << " * " << info_.c_str();
                error_ = 0;
            }
            std::cout << " error is Infinity" << std::endl;
        }

        integral_ += error_ * dt;

        double raw_derivative = (error_ - prev_error_) / dt;
        derivative_ = (tau_ / (tau_ + dt)) * prev_derivative_ + (dt / (tau_ + dt)) * raw_derivative;

        double proportional_term = kp_ * error_;
        double integral_term = ki_ * integral_;
        double derivative_term = kd_ * derivative_;

        control_output_raw_ = proportional_term + integral_term + derivative_term;
        double control_output_ = control_output_raw_;
        control_saturated_ = false;

        if (control_output_raw_ > umax_)
        {
            control_output_ = umax_;
            integral_ -= error_ * dt;
            control_saturated_ = true;
        }
        else if (control_output_raw_ < umin_)
        {
            control_output_ = umin_;
            integral_ -= error_ * dt;
            control_saturated_ = true;
        }

        prev_error_ = error_;
        prev_derivative_ = derivative_;

        return control_output_;
    }

    bool AnalogPID::is_saturated(void)
    {
        return control_saturated_;
    }

    double AnalogPID::get_unsaturated_output(void)
    {
        return control_output_raw_;
    }

    double AnalogPID::get_error(void)
    {
        return prev_error_;
    }

    double AnalogPID::get_derivative_error(void)
    {
        return derivative_;
    }

    double AnalogPID::get_integral_error(void)
    {
        return integral_;
    }

    void AnalogPID::reset()
    {
        integral_ = 0.0;
        prev_error_ = 0.0;
        derivative_ = 0.0;
        prev_derivative_ = 0.0;

        control_output_raw_ = 0.0;
        control_saturated_ = false;
    }

    double AnalogPID::get_gain_kp(void)
    {
        return kp_;
    }

    void AnalogPID::set_gain_kp(const double kp)
    {
        kp_ = kp;

        if (!info_.empty())
        {
            std::cout << " * " << info_.c_str();
        }
        std::cout << " gain Kp: " << kp_ << std::endl;
    }

    double AnalogPID::get_gain_ki(void)
    {
        return ki_;
    }

    void AnalogPID::set_gain_ki(const double ki)
    {
        ki_ = ki;

        if (!info_.empty())
        {
            std::cout << " * " << info_.c_str();
        }
        std::cout << " gain Ki: " << ki_ << std::endl;
    }

    double AnalogPID::get_gain_kd(void)
    {
        return kd_;
    }

    void AnalogPID::set_gain_kd(const double kd)
    {
        kd_ = kd;

        if (!info_.empty())
        {
            std::cout << " * " << info_.c_str();
        }
        std::cout << " gain Kd: " << kd_ << std::endl;
    }

    double AnalogPID::get_filter_coeficient(void)
    {
        return tau_;
    }

    void AnalogPID::set_filter_coeficient(const double tau)
    {
        tau_ = tau;

        if (!info_.empty())
        {
            std::cout << " * " << info_.c_str();
        }
        std::cout << " filter coeficient: " << tau_ << std::endl;
    }

    std::vector<double> AnalogPID::get_output_saturation(void)
    {
        std::vector<double> saturation;
        saturation.push_back(umin_);
        saturation.push_back(umax_);

        if (!info_.empty())
        {
            std::cout << " * " << info_.c_str();
        }
        std::cout << " saturation [min | max]: " << umin_ << " | " << umax_ << std::endl;
        return saturation;
    }

    void AnalogPID::set_output_saturation(const double umin, const double umax)
    {
        umin_ = umin;
        umax_ = umax;
    }

    std::string AnalogPID::get_info(void)
    {
        return info_;
    }

    void AnalogPID::set_info(std::string info)
    {
        info_ = info;
        std::cout << "Controller information: " << info_ << std::endl;
    }
}