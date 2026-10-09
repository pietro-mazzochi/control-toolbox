// Copyright (c) 2024, ISI SIM
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include <iostream>
#include <cmath>

#include "pid_controllers/digital_pid_controller.hpp"

namespace PIDControllers
{
    DigitalPID::DigitalPID(const double kp,
                           const double ki,
                           const double kd,
                           const double dt,
                           const double umin,
                           const double umax,
                           const double N,
                           std::optional<std::string> info) : control_output_raw_(0.0),
                                                              control_saturated_(false)
    {
        set_info(info.value_or(""));
        set_gain_kp(kp);
        set_gain_ki(ki);
        set_gain_kd(kd);
        set_time_step(dt);
        set_output_saturation(umin, umax);
        set_filter_coeficient(N);
        reset();
    }

    double DigitalPID::compute(double setpoint, double measured_value)
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

        integral_ += error_ * dt_;

        double raw_derivative = (error_ - prev_error_) / dt_;
        derivative_ = (N_ * raw_derivative + prev_derivative_) / (1 + N_ * dt_);

        double proportional_term = kp_ * error_;
        double integral_term = ki_ * integral_;
        double derivative_term = kd_ * derivative_;

        control_output_raw_ = proportional_term + integral_term + derivative_term;
        double control_output_ = control_output_raw_;
        control_saturated_ = false;

        if (control_output_raw_ > umax_)
        {
            control_output_ = umax_;
            integral_ -= error_ * dt_;
            control_saturated_ = true;
        }
        else if (control_output_raw_ < umin_)
        {
            control_output_ = umin_;
            integral_ -= error_ * dt_;
            control_saturated_ = true;
        }

        prev_error_ = error_;
        prev_derivative_ = derivative_;

        return control_output_;
    }

    bool DigitalPID::is_saturated(void)
    {
        return control_saturated_;
    }

    double DigitalPID::get_unsaturated_output(void)
    {
        return control_output_raw_;
    }

    double DigitalPID::get_error(void)
    {
        return prev_error_;
    }

    double DigitalPID::get_derivative_error(void)
    {
        return derivative_;
    }

    double DigitalPID::get_integral_error(void)
    {
        return integral_;
    }

    void DigitalPID::reset()
    {
        integral_ = 0.0;
        prev_error_ = 0.0;
        prev_measured_ = 0.0;
        derivative_ = 0.0;
        prev_derivative_ = 0.0;

        control_output_raw_ = 0.0;
        control_saturated_ = false;
    }

    double DigitalPID::get_gain_kp(void)
    {
        return kp_;
    }

    void DigitalPID::set_gain_kp(const double kp)
    {
        kp_ = kp;

        if (!info_.empty())
        {
            std::cout << " * " << info_.c_str();
        }
        std::cout << " Gain Kp: " << kp_ << std::endl;
    }

    double DigitalPID::get_gain_ki(void)
    {
        return ki_;
    }

    void DigitalPID::set_gain_ki(const double ki)
    {
        ki_ = ki;

        if (!info_.empty())
        {
            std::cout << " * " << info_.c_str();
        }
        std::cout << " Gain Ki: " << ki_ << std::endl;
    }

    double DigitalPID::get_gain_kd(void)
    {
        return kd_;
    }

    void DigitalPID::set_gain_kd(const double kd)
    {
        kd_ = kd;

        if (!info_.empty())
        {
            std::cout << " * " << info_.c_str();
        }
        std::cout << " Gain Kd: " << kd_ << std::endl;
    }

    double DigitalPID::get_time_step(void)
    {
        return this->dt_;
    }

    void DigitalPID::set_time_step(const double dt)
    {
        this->dt_ = dt;

        if (!info_.empty())
        {
            std::cout << " * " << info_.c_str();
        }
        std::cout << " time step (ms): " << (this->dt_ * 1000) << std::endl;
    }

    double DigitalPID::get_filter_coeficient(void)
    {
        return N_;
    }

    void DigitalPID::set_filter_coeficient(const double N)
    {
        N_ = N;

        if (!info_.empty())
        {
            std::cout << " * " << info_.c_str();
        }
        std::cout << " filter coeficient: " << N_ << std::endl;
    }

    std::vector<double> DigitalPID::get_output_saturation(void)
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

    void DigitalPID::set_output_saturation(const double umin, const double umax)
    {
        umin_ = umin;
        umax_ = umax;
    }

    std::string DigitalPID::get_info(void)
    {
        return info_;
    }

    void DigitalPID::set_info(std::string info)
    {
        info_ = info;
        std::cout << "Controller information: " << info_ << std::endl;
    }

}