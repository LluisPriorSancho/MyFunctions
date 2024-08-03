"""
Module that implements different functions for ROS2 projects.
"""
import math as m
from geometry_msgs.msg import Vector3, Point, Quaternion

class MyROSFunctions:
    def __init__(self) -> None:
        """
        This class implements basic functions related to the ROS environments.
        """

    @staticmethod
    def vector3_to_point(vector3: Vector3) -> Point:
        """ Transformation between variable types.

        Args:
            vector3 (Vector3): variable to transform.

        Returns:
            Point: transformed variable.
        """
        return Point(x=vector3.x, y=vector3.y, z=vector3.z)
    
    @staticmethod
    def point_to_vector3(point: Point) -> Vector3:
        """ Transformation between variable types.

        Args:
            point (Point): variable to transform.

        Returns:
            Vector3: transformed variable.
        """
        return Vector3(x=point.x, y=point.y, z=point.z)
    
    @staticmethod
    def get_direction(initial_point: Point, 
                      final_point: Point, 
                      bias_ip: Vector3 = Vector3(), 
                      bias_fp: Vector3 = Vector3()) -> Vector3:
        """ Gets the direction vector from one point to another. Can include an offset to those points.

        Args:
            initial_point (Point): Initial point of the direction vector.
            final_point (Point): Final point of the direction vector.
            bias_ip (Vector3, optional): Bias offset to the initial point. Defaults to Vector3().
            bias_fp (Vector3, optional): Bias offset to the final point. Defaults to Vector3().
            

        Returns:
            Vector3: Direction vector.
        """
        direction = Vector3()
        direction.x = final_point.x + bias_fp.x - initial_point.x - bias_ip.x
        direction.y = final_point.y + bias_fp.y - initial_point.y - bias_ip.y
        direction.z = final_point.z + bias_fp.z - initial_point.z - bias_ip.z
        return direction
        

    @staticmethod
    def get_distance(initial_point: Point, 
                     final_point: Point, 
                     bias_ip: Vector3 = Vector3(), 
                     bias_fp: Vector3 = Vector3()) -> float:
        """ Computes de distance between points in 3D.

        Args:
            initial_point (Point): Initial point of the direction vector.
            final_point (Point): Final point of the direction vector.
            bias_ip (Vector3, optional): Bias offset to the initial point. Defaults to Vector3().
            bias_fp (Vector3, optional): Bias offset to the final point. Defaults to Vector3().

        Returns:
            float: Computed distance
        """
        direction = MyROSFunctions.get_direction(initial_point, final_point, bias_ip, bias_fp)
        distance = m.sqrt(direction.x**2 + direction.y**2 + direction.z**2)
        return distance
    
    @staticmethod
    def quaternion_to_euler(in_quaternion: bool,
                            quaternion: Quaternion = Quaternion(),
                            x: float = 0.0, 
                            y: float = 0.0, 
                            z: float = 0.0, 
                            w: float = 0.0) -> tuple[float, float, float]:
        """ Convert a quaternion into euler angles. Returns clockwise.

        Args:
            in_quaternion(bool): Indicates if the input is in geometry_msgs.quaternion variable type.
            quaternion(Quaternion, optional): Input if the variable is in geometry_msgs.quaternion variable type.
            x (float, optional): Quaternion x component.
            y (float, optional): Quaternion y component.
            z (float, optional): Quaternion z component.
            w (float, optional): Quaternion w component.

        Returns:
            tuple[float, float, float]: roll, pitch, yaw in radians
        """
        if in_quaternion:
            x, y, z, w = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = m.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = m.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = m.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z 
    
    @staticmethod
    def euler_to_quaternion(roll: float,
                            pitch: float,
                            yaw: float) -> tuple[float, float, float, float]:
        """ Convert euler angles (ZYX) into quaternion.

        Args:
            roll (float): rotation around x-axis in radians.
            pitch (float): rotation around y-axis in radians.
            yaw (float): rotation around z-axis in radians.

        Returns:
            tuple[float, float, float, float]: The orientation in quaternion [x,y,z,w] format.
        """
        cr = m.cos(roll * 0.5)
        sr = m.sin(roll * 0.5)
        cp = m.cos(pitch * 0.5)
        sp = m.sin(pitch * 0.5)
        cy = m.cos(yaw * 0.5)
        sy = m.sin(yaw * 0.5)
        
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw
    
    @staticmethod
    def radians_to_degrees(radians):
        """ Convert radians to degrees.

        Args:
            radians (float or list of floats): Angle(s) in radians.

        Returns:
            float or list of floats: Angle(s) in degrees.
        """
        if isinstance(radians, (list, tuple)):  # Handle lists or tuples
            return [r * 180 / m.pi for r in radians]
        else:  # Handle single float
            return radians * 180 / m.pi
        
    @staticmethod
    def degrees_to_radians(degrees):
        """ Convert degrees to radians.

        Args:
            degrees (float or list of floats): Angle(s) in degrees.

        Returns:
            float or list of floats: Angle(s) in radians.
        """
        if isinstance(degrees, (list, tuple)):  # Handle lists or tuples
            return [d * m.pi / 180 for d in degrees]
        else:  # Handle single float
            return degrees * m.pi / 180