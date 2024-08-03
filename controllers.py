"""
Module that implements different types of controllers.
"""
import math as m

class BaseFunctions:
    def __init__(self, up_range: float = m.pi, 
                       bot_range: float = -m.pi) -> None:
        """ This class implements basic functions for the different controllers. Inputs common parameters for the controllers.

        Args:
            up_range (float, optional): Maximum value of the angle limitation. Defaults to m.pi.
            bot_range (float, optional): Minimum value of the angle limitation. Defaults to -m.pi.
        """
        self.up_range = up_range
        self.bot_range = bot_range
        

    def set_limits_angle(self, new_up: float, new_bot: float) -> None:
        """ Sets the range values for limiting angles.

        Args:
            new_up (float): New upper limit.
            new_bot (float): New lower limit.

        Raises:
            ValueError: Upper limit must be bigger than the lower limit.
        """
        if new_up < new_bot:
            raise ValueError(f"Error: new up value ({new_up}) must be greater than bot value ({new_bot}).")
        self.bot_range = new_bot
        self.up_range = new_up

    def limit_range_angle(self, value: float) -> float:
        """ Returns an angle limited to the range [bot_range, up_range].

        Args:
            value (float): Value to limit.

        Returns:
            float: Limited value.
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
        """This class implements a PID controller. It allows also the control of an angle.

        Args:
            angle (bool, optional): Implements limits to the range [-pi, pi]. To modify limits call set_limits_angle(). Defaults to False.
            kp (float, optional): Proportional gain. Defaults to 1.0.
            ki (float, optional): Integral gain. Defaults to 0.0.
            kd (float, optional): Derivative gain. Defaults to 0.0.
        """
        super().__init__()  # Initialize the base class with default ranges
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.angle = angle
        self.prev_error = 0
        self.integral = 0

    @staticmethod
    def get_PID_params(controller_type: str = "P",
                       T: float = 0.0, 
                       L: float = 0.0) -> tuple[float, float, float]:
        """ Computes the Ziegler-Nichols PID parameters.

        Args:
            controller_type (str, optional): type of controller: P, PI or PID. Defaults to "P".
            T (float, optional): Defaults to 0.0.
            L (float, optional): Defaults to 0.0.

        Raises:
            ValueError: T and L must be different.

        Returns:
            tuple[float, float, float]: values of Kp, Ki and Kd.
        """
        if T == L:
            raise ValueError(f"Error: T ({T}) must be different from L ({L}).")
        if controller_type == "P":
            return T/L, m.inf, 0.0
        elif controller_type == "PI":
            return 0.9*T/L, L/0.3, 0
        else: return 1.2*T/L, 2*L, 0.5*L


    def calc(self, state: float, 
                   target: float, 
                   dt: float, 
                   bias: float = 0.0) -> float:
        """ Computes one step of the controller.

        Args:
            state (float): Actual value of the controlled variable.
            target (float): Goal value of the controlled variable.
            dt (float): Time step.
            bias (float, optional): Offset added to the controlled action. Defaults to 0.0.

        Returns:
            float: New control action.
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
        """ This class implements an ON/OFF controller. It allows also the control of an angle. 

        Args:
            angle (bool, optional): Implements limits to the range [-pi, pi]. To modify limits call set_limits_angle(). Defaults to False.
            hysteresis (float, optional): Value of the hysteresis. Defaults to 0.0.
            prev_control (bool, optional): Previious control action. Defaults to False.
        """
        super().__init__()  # Initialize the base class with default ranges
        self.hysteresis = hysteresis
        self.angle = angle
        self.prev_control = prev_control

    def calc(self, state: float, 
                   target: float) -> bool:
        """ Computes one step of the controller. 

        Args:
            state (float): Actual value of the controlled variable.
            target (float): Goal value of the controlled variable.

        Returns:
            bool: Returns True/False depending on the state of action.
        """
        if self.angle:
            state = self.limit_range_angle(state)
        if (state >= (target + self.hysteresis)) and self.prev_control:
            self.prev_control = False
        elif (state <= (target - self.hysteresis)) and not self.prev_control:
            self.prev_control = True
        return self.prev_control
