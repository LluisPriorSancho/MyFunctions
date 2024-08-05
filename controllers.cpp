#include "controllers.hpp"
#include <cmath>
#include <stdexcept>
#include <tuple>
#include <limits>

BaseFunctions::BaseFunctions(double up_range, double bot_range)
    : up_range(up_range), bot_range(bot_range) {}

    void BaseFunctions::set_limits_angle(double new_up, double new_bot) {
        if (new_up < new_bot) {
            throw std::invalid_argument("Error: new up value must be greater than bot value.");
        }
        bot_range = new_bot;
        up_range = new_up;
    }

    double BaseFunctions::limit_range_angle(double value) const {
        while (value < bot_range) {
            value += 2 * M_PI;
        }
        while (value > up_range) {
            value -= 2 * M_PI;
        }
        return value;
    }

PIDController::PIDController(bool angle, double kp, double ki, double kd)
    : BaseFunctions(), kp(kp), ki(ki), kd(kd), angle(angle), prev_error(0.0), integral(0.0) {}

    std::tuple<double, double, double> PIDController::get_PID_params(const std::string& controller_type, double T, double L) {
        if (T == L) {
            throw std::invalid_argument("Error: T must be different from L.");
        }
        if (controller_type == "P") {
            return std::make_tuple(T / L, std::numeric_limits<double>::infinity(), 0.0);
        } else if (controller_type == "PI") {
            return std::make_tuple(0.9 * T / L, L / 0.3, 0.0);
        } else {
            return std::make_tuple(1.2 * T / L, 2 * L, 0.5 * L);
        }
    }

    double PIDController::calc(double state, double target, double dt, double bias) {
        double error = target - state;
        if (angle) {
            error = limit_range_angle(error);
        }
        integral += error * dt;
        double derivative = (error - prev_error) / dt;
        prev_error = error;
        return kp * error + ki * integral + kd * derivative + bias;
    }

OnOffController::OnOffController(bool angle, double hysteresis, bool prev_control)
    : BaseFunctions(), hysteresis(hysteresis), angle(angle), prev_control(prev_control) {}

    bool OnOffController::calc(double state, double target) {
        if (angle) {
            state = limit_range_angle(state);
        }
        if ((state >= (target + hysteresis)) && prev_control) {
            prev_control = false;
        } else if ((state <= (target - hysteresis)) && !prev_control) {
            prev_control = true;
        }
        return prev_control;
    }