#pragma once


#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <webots/robot.h>
#include <webots/inertial_unit.h>
#include <webots/gyro.h>
#include <webots/accelerometer.h>
#include <webots/gps.h>
#include <webots/range_finder.h>
#include <webots/lidar.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>

#include <vector>


static constexpr int TIME_STEP = 32;

// 运动学接口
class Kinematics {
public:
    virtual ~Kinematics() = default;
    virtual void setVelocity(double vx, double vy, double omega) = 0;
    virtual void initJointState(sensor_msgs::msg::JointState& joint_state_msg_) = 0;
    virtual void updateJointState(sensor_msgs::msg::JointState& joint_state_msg_) = 0;
    virtual void updateOdometryTwist(geometry_msgs::msg::Twist& twist) = 0;
private:

};


class WebotsDriver : public rclcpp::Node {
public:
    explicit WebotsDriver();
    ~WebotsDriver() override;
    void publishIMU();
    void publishOdometryAndJointState();
    void publish3DPoints();
    void publishTof();
    void publishClock();
    void run();

protected:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void updateLoop();

private:

    // Webots 设备
    WbDeviceTag imu_{};
    WbDeviceTag gyro_{};
    WbDeviceTag accelerometer_{};
    WbDeviceTag gps_{};
    WbDeviceTag lidar_3d_ {};
    WbDeviceTag tof_ {};

    // 运动学（解耦！可替换：四舵轮 / 差速 / 麦克纳姆）
    std::unique_ptr<Kinematics> kinematics_;

    // ROS 接口
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_3d_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr tof_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr tof_camera_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::JointState joint_state_msg_;
};