#include "my_ros_package/my_ros_functions.hpp"

geometry_msgs::msg::Point MyROSFunctions::vector3_to_point(const geometry_msgs::msg::Vector3& vector3) {
    geometry_msgs::msg::Point point;
    point.x = vector3.x;
    point.y = vector3.y;
    point.z = vector3.z;
    return point;
}

geometry_msgs::msg::Vector3 MyROSFunctions::point_to_vector3(const geometry_msgs::msg::Point& point) {
    geometry_msgs::msg::Vector3 vector3;
    vector3.x = point.x;
    vector3.y = point.y;
    vector3.z = point.z;
    return vector3;
}

geometry_msgs::msg::Vector3 MyROSFunctions::get_direction(const geometry_msgs::msg::Point& initial_point, 
                                                          const geometry_msgs::msg::Point& final_point,
                                                          const geometry_msgs::msg::Vector3& bias_ip,
                                                          const geometry_msgs::msg::Vector3& bias_fp) {
    geometry_msgs::msg::Vector3 direction;
    direction.x = final_point.x + bias_fp.x - initial_point.x - bias_ip.x;
    direction.y = final_point.y + bias_fp.y - initial_point.y - bias_ip.y;
    direction.z = final_point.z + bias_fp.z - initial_point.z - bias_ip.z;
    return direction;
}

double MyROSFunctions::get_distance(const geometry_msgs::msg::Point& initial_point, 
                                    const geometry_msgs::msg::Point& final_point,
                                    const geometry_msgs::msg::Vector3& bias_ip,
                                    const geometry_msgs::msg::Vector3& bias_fp) {
    geometry_msgs::msg::Vector3 direction = get_direction(initial_point, final_point, bias_ip, bias_fp);
    return std::sqrt(direction.x * direction.x + direction.y * direction.y + direction.z * direction.z);
}

std::tuple<double, double, double> MyROSFunctions::quaternion_to_euler(bool in_quaternion,
                                                                        const geometry_msgs::msg::Quaternion& quaternion,
                                                                        double x, double y, double z, double w) {
    if (in_quaternion) {
        x = quaternion.x;
        y = quaternion.y;
        z = quaternion.z;
        w = quaternion.w;
    }

    double t0 = +2.0 * (w * x + y * z);
    double t1 = +1.0 - 2.0 * (x * x + y * y);
    double roll_x = std::atan2(t0, t1);

    double t2 = +2.0 * (w * y - z * x);
    t2 = std::min(std::max(t2, -1.0), 1.0);
    double pitch_y = std::asin(t2);

    double t3 = +2.0 * (w * z + x * y);
    double t4 = +1.0 - 2.0 * (y * y + z * z);
    double yaw_z = std::atan2(t3, t4);

    return {roll_x, pitch_y, yaw_z};
}

std::tuple<double, double, double, double> MyROSFunctions::euler_to_quaternion(double roll, double pitch, double yaw) {
    double cr = std::cos(roll * 0.5);
    double sr = std::sin(roll * 0.5);
    double cp = std::cos(pitch * 0.5);
    double sp = std::sin(pitch * 0.5);
    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);

    double qw = cr * cp * cy + sr * sp * sy;
    double qx = sr * cp * cy - cr * sp * sy;
    double qy = cr * sp * cy + sr * cp * sy;
    double qz = cr * cp * sy - sr * sp * cy;
    return {qx, qy, qz, qw};
}

std::vector<double> MyROSFunctions::radians_to_degrees(const std::vector<double>& radians) {
    std::vector<double> degrees;
    degrees.reserve(radians.size());
    for (double rad : radians) {
        degrees.push_back(rad * 180.0 / M_PI);
    }
    return degrees;
}

std::vector<double> MyROSFunctions::degrees_to_radians(const std::vector<double>& degrees) {
    std::vector<double> radians;
    radians.reserve(degrees.size());
    for (double deg : degrees) {
        radians.push_back(deg * M_PI / 180.0);
    }
    return radians;
}
