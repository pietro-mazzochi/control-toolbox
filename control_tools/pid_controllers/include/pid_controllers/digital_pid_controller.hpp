// Copyright (c) 2024, ISI SIM
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#ifndef PID_CONTROLLERS__DIGITAL_PID_CONTROLLER_HPP_
#define PID_CONTROLLERS__DIGITAL_PID_CONTROLLER_HPP_

#include <vector>
#include <string>
#include <memory>
#include <optional>

enum
{
    DIGITAL_PID_KP_INDEX = 0,
    DIGITAL_PID_KI_INDEX,
    DIGITAL_PID_KD_INDEX,
    DIGITAL_PID_UMIN_INDEX,
    DIGITAL_PID_UMAX_INDEX,
    DIGITAL_PID_N_INDEX,
    DIGITAL_PID_DT_INDEX,
    DIGITAL_PID_INDEX_SIZE
};

namespace PIDControllers
{

    class DigitalPID
    {
    public:
        DigitalPID(const double kp,
                   const double ki,
                   const double kd,
                   const double dt,
                   const double umin,
                   const double umax,
                   const double N,
                   std::optional<std::string> info = std::nullopt);
        double compute(const double setpoint, const double measured_value);

        bool is_saturated(void);
        double get_unsaturated_output(void);

        double get_error(void);
        double get_derivative_error(void);
        double get_integral_error(void);
        void reset();

        double get_gain_kp(void);
        void set_gain_kp(const double kp);

        double get_gain_ki(void);
        void set_gain_ki(const double ki);

        double get_gain_kd(void);
        void set_gain_kd(const double kd);

        double get_time_step(void);
        void set_time_step(const double dt);

        double get_filter_coeficient(void);
        void set_filter_coeficient(const double N);

        std::vector<double> get_output_saturation(void);
        void set_output_saturation(const double umin, const double umax);

        std::string get_info(void);
        void set_info(std::string info);

    private:
        double kp_, ki_, kd_, dt_, umax_, umin_, N_;
        double integral_, prev_error_, prev_measured_, derivative_, prev_derivative_;
        std::string info_;

        double control_output_raw_;
        bool control_saturated_;
    };
    using DigitalPidControllerPtr = std::shared_ptr<DigitalPID>;

}

#endif // PID_CONTROLLERS__DIGITAL_PID_CONTROLLER_HPP_
