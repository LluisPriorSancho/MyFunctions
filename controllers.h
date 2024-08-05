#ifndef CONTROLLERS_H
#define CONTROLLERS_H

#include <cmath>
#include <stdexcept>
#include <tuple>
#include <limits>
#include <string>

/**
 * @brief BaseFunctions provides basic functionality for controllers.
 * 
 * This class implements basic functions for the different controllers, including
 * setting and limiting angle ranges.
 */
class BaseFunctions {
public:
    /**
     * @brief Construct a new BaseFunctions object.
     * 
     * @param up_range Maximum value of the angle limitation. Defaults to M_PI.
     * @param bot_range Minimum value of the angle limitation. Defaults to -M_PI.
     */
    BaseFunctions(double up_range = M_PI, double bot_range = -M_PI);

    /**
     * @brief Set the limits for angle range.
     * 
     * Sets new upper and lower limits for the angle range. The upper limit must be
     * greater than the lower limit.
     * 
     * @param new_up New upper limit of the angle range.
     * @param new_bot New lower limit of the angle range.
     * 
     * @throws std::invalid_argument if new_up is less than new_bot.
     */
    void set_limits_angle(double new_up, double new_bot);

    /**
     * @brief Limit the angle to the specified range.
     * 
     * Adjusts the angle value to ensure it falls within the range [bot_range, up_range].
     * 
     * @param value The angle value to be limited.
     * @return The adjusted angle value within the specified range.
     */
    double limit_range_angle(double value) const;

protected:
    double up_range; /**< Maximum value of the angle range. */
    double bot_range; /**< Minimum value of the angle range. */
};

/**
 * @brief PIDController implements a PID (Proportional-Integral-Derivative) controller.
 * 
 * This class allows the control of a variable using PID control. It also supports
 * angle limitation if the `angle` parameter is set to true.
 */
class PIDController : public BaseFunctions {
public:
    /**
     * @brief Construct a new PIDController object.
     * 
     * @param angle If true, limits the control action to the range [-pi, pi]. Defaults to false.
     * @param kp Proportional gain. Defaults to 1.0.
     * @param ki Integral gain. Defaults to 0.0.
     * @param kd Derivative gain. Defaults to 0.0.
     */
    PIDController(bool angle = false, double kp = 1.0, double ki = 0.0, double kd = 0.0);

    /**
     * @brief Get PID parameters using the Ziegler-Nichols method.
     * 
     * Computes the PID parameters based on the Ziegler-Nichols tuning method. The returned
     * values depend on the type of controller specified.
     * 
     * @param controller_type Type of the controller: "P", "PI", or "PID". Defaults to "P".
     * @param T The ultimate period (used in Ziegler-Nichols method). Defaults to 0.0.
     * @param L The ultimate gain (used in Ziegler-Nichols method). Defaults to 0.0.
     * 
     * @return A tuple containing the computed Kp, Ki, and Kd values.
     * 
     * @throws std::invalid_argument if T is equal to L.
     */
    static std::tuple<double, double, double> get_PID_params(const std::string& controller_type = "P",
                                                             double T = 0.0, double L = 0.0);

    /**
     * @brief Compute the control action based on the PID algorithm.
     * 
     * Calculates the control action by computing the PID output based on the current
     * state, target value, time step, and an optional bias.
     * 
     * @param state The current value of the controlled variable.
     * @param target The target value of the controlled variable.
     * @param dt The time step between control updates.
     * @param bias An optional offset added to the control action. Defaults to 0.0.
     * @return The computed control action.
     */
    double calc(double state, double target, double dt, double bias = 0.0);

private:
    double kp; /**< Proportional gain. */
    double ki; /**< Integral gain. */
    double kd; /**< Derivative gain. */
    bool angle; /**< If true, limits the angle range. */
    double prev_error; /**< Previous error value for derivative calculation. */
    double integral; /**< Accumulated integral value. */
};

/**
 * @brief OnOffController implements an ON/OFF controller.
 * 
 * This class implements a simple ON/OFF control algorithm. It supports angle
 * limitation if the `angle` parameter is set to true.
 */
class OnOffController : public BaseFunctions {
public:
    /**
     * @brief Construct a new OnOffController object.
     * 
     * @param angle If true, limits the control action to the range [-pi, pi]. Defaults to false.
     * @param hysteresis The hysteresis value to prevent rapid switching. Defaults to 0.0.
     * @param prev_control The previous control action state. Defaults to false.
     */
    OnOffController(bool angle = false, double hysteresis = 0.0, bool prev_control = false);

    /**
     * @brief Compute the control action based on the ON/OFF algorithm.
     * 
     * Determines the control action based on the current state, target value, and hysteresis.
     * 
     * @param state The current value of the controlled variable.
     * @param target The target value of the controlled variable.
     * @return The new control action state (true or false).
     */
    bool calc(double state, double target);

private:
    double hysteresis; /**< Hysteresis value to prevent rapid switching. */
    bool angle; /**< If true, limits the angle range. */
    bool prev_control; /**< Previous control action state. */
};

#endif // CONTROLLERS_H

