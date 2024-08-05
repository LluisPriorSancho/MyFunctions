#ifndef MY_ROS_FUNCTIONS_H
#define MY_ROS_FUNCTIONS_H

#include <cmath>
#include <tuple>
#include <vector>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

/**
 * @brief MyROSFunctions provides utility functions related to ROS environments.
 */
class MyROSFunctions {
public:
    MyROSFunctions() = default;

    /**
     * @brief Transform geometry_msgs::msg::Vector3 to geometry_msgs::msg::Point.
     * @param vector3 The Vector3 to transform.
     * @return Transformed Point.
     */
    static geometry_msgs::msg::Point vector3_to_point(const geometry_msgs::msg::Vector3& vector3);

    /**
     * @brief Transform geometry_msgs::msg::Point to geometry_msgs::msg::Vector3.
     * @param point The Point to transform.
     * @return Transformed Vector3.
     */
    static geometry_msgs::msg::Vector3 point_to_vector3(const geometry_msgs::msg::Point& point);

    /**
     * @brief Get the direction vector from one point to another with optional bias.
     * @param initial_point Initial point of the direction vector.
     * @param final_point Final point of the direction vector.
     * @param bias_ip Bias offset to the initial point.
     * @param bias_fp Bias offset to the final point.
     * @return Direction vector.
     */
    static geometry_msgs::msg::Vector3 get_direction(const geometry_msgs::msg::Point& initial_point, 
                                                     const geometry_msgs::msg::Point& final_point,
                                                     const geometry_msgs::msg::Vector3& bias_ip = geometry_msgs::msg::Vector3(),
                                                     const geometry_msgs::msg::Vector3& bias_fp = geometry_msgs::msg::Vector3());

    /**
     * @brief Compute the distance between two points with optional bias.
     * @param initial_point Initial point.
     * @param final_point Final point.
     * @param bias_ip Bias offset to the initial point.
     * @param bias_fp Bias offset to the final point.
     * @return Computed distance.
     */
    static double get_distance(const geometry_msgs::msg::Point& initial_point, 
                               const geometry_msgs::msg::Point& final_point,
                               const geometry_msgs::msg::Vector3& bias_ip = geometry_msgs::msg::Vector3(),
                               const geometry_msgs::msg::Vector3& bias_fp = geometry_msgs::msg::Vector3());

    /**
     * @brief Convert a quaternion to Euler angles.
     * @param in_quaternion Indicates if input is in Quaternion type.
     * @param quaternion Input quaternion value.
     * @param x Quaternion x component.
     * @param y Quaternion y component.
     * @param z Quaternion z component.
     * @param w Quaternion w component.
     * @return Roll, pitch, and yaw in radians.
     */
    static std::tuple<double, double, double> quaternion_to_euler(bool in_quaternion,
                                                                   const geometry_msgs::msg::Quaternion& quaternion = geometry_msgs::msg::Quaternion(),
                                                                   double x = 0.0, double y = 0.0, double z = 0.0, double w = 0.0);

    /**
     * @brief Convert Euler angles to a quaternion.
     * @param roll Rotation around x-axis in radians.
     * @param pitch Rotation around y-axis in radians.
     * @param yaw Rotation around z-axis in radians.
     * @return Quaternion [x, y, z, w].
     */
    static std::tuple<double, double, double, double> euler_to_quaternion(double roll, double pitch, double yaw);

    /**
     * @brief Convert radians to degrees.
     * @param radians Angle(s) in radians.
     * @return Angle(s) in degrees.
     */
    static std::vector<double> radians_to_degrees(const std::vector<double>& radians);

    /**
     * @brief Convert degrees to radians.
     * @param degrees Angle(s) in degrees.
     * @return Angle(s) in radians.
     */
    static std::vector<double> degrees_to_radians(const std::vector<double>& degrees);
};

#endif // MY_ROS_FUNCTIONS_H
