"""
Module that implements different functions for ROS2 projects.
"""
import math as m
from geometry_msgs.msg import Vector3, Point

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