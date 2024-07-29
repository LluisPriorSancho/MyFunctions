"""
Module that implements different types of controllers
"""
import math as m

class BaseFunctions:
    def __init__(self, up_range: float = m.pi, 
                       bot_range: float = -m.pi) -> None:
        """
        This class implements basic functions for the different controllers. Inputs common parameters for the controllers

            - up_range: Maximum value of the angle
            - bot_range: Minimum value of the angle
        """
        self.up_range = up_range
        self.bot_range = bot_range

    def set_limits_angle(self, new_up: float, new_bot: float) -> None:
        """
        Sets the range values for limiting angles
        """
        self.bot_range = new_bot
        self.up_range = new_up

    def limit_range_angle(self, value: float) -> float:
        """
        Returns an angle limited to the range [bot_range, up_range]
        """
        while value < self.bot_range:
            value += 2 * m.pi
        while value > self.up_range:
            value -= 2 * m.pi
        return value

class PIDController(BaseFunctions):
    def __init__(self, angle: bool = False, 
                       kp: float = 1.0, 
                       ki: float = 0.0, 
                       kd: float = 0.0) -> None:
        """
        This class implements a PID controller. It allows also the control of an angle.

            - angle: Implements limits to the range [-pi, pi]. To modify limits call set_limits_angle()
            - kp: Proportional gain
            - ki: Integral gain
            - kd: Derivative gain
        """
        super().__init__()  # Initialize the base class with default ranges
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.angle = angle
        self.prev_error = 0
        self.integral = 0

    def calc(self, state: float, 
                   target: float, 
                   dt: float, 
                   bias: float = 0.0) -> float:
        """
        Computes one step of the controller.

            - state: Actual value of the controlled variable
            - target: Goal value of the controlled variable
            - dt: Time step
            - bias: Offset added to the controlled action
        """
        error = target - state
        if self.angle:
            error = self.limit_range_angle(error)
        self.integral += (error * dt)
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative + bias

class OnOffController(BaseFunctions):
    def __init__(self, angle: bool = False,
                       hysteresis: float = 0.0,
                       prev_control: bool = False) -> None:
        """
        This class implements an ON/OFF controller. It allows also the control of an angle. Returns True/False depending on the state of action

            - angle: Implements limits to the range [-pi, pi]. To modify limits call set_limits_angle()
            - hysteresis: Value of the hysteresis
        """
        super().__init__()  # Initialize the base class with default ranges
        self.hysteresis = hysteresis
        self.angle = angle
        self.prev_control = prev_control

    def calc(self, state: float, 
                   target: float) -> bool:
        """
        Computes one step of the controller. 

            - state: Actual value of the controlled variable
            - target: Goal value of the controlled variable
        """
        if self.angle:
            state = self.limit_range_angle(state)
        if (state >= (target + self.hysteresis)) and self.prev_control:
            self.prev_control = False
        elif (state <= (target - self.hysteresis)) and not self.prev_control:
            self.prev_control = True
        return self.prev_control
