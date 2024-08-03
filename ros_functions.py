"""
Module that implements different functions for ROS2 projects
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
        """ Transformation between variable types

        Args:
            vector3 (Vector3): variable to transform

        Returns:
            Point: transformed variable
        """
        return Point(x=vector3.x, y=vector3.y, z=vector3.z)
    
    @staticmethod
    def point_to_vector3(point: Point) -> Vector3:
        """ Transformation between variable types

        Args:
            point (Point): variable to transform

        Returns:
            Vector3: transformed variable
        """
        return Vector3(x=point.x, y=point.y, z=point.z)
