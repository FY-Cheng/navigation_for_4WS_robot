#pragma once


#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <webots/robot.h>
#include <webots/inertial_unit.h>
#include <webots/gps.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>


static constexpr int TIME_STEP = 32;

// 运动学接口
class Kinematics {
public:
    virtual ~Kinematics() = default;
    virtual void setVelocity(double vx, double vy, double omega) = 0;
};


class WebotsDriver : public rclcpp::Node {
public:
    explicit WebotsDriver();
    ~WebotsDriver() override;

protected:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void updateLoop();
    void publishIMU();
    void publishOdometry();

private:

    // Webots 设备
    WbDeviceTag imu_{};
    WbDeviceTag gps_{};

    // 运动学（解耦！可替换：四舵轮 / 差速 / 麦克纳姆）
    std::unique_ptr<Kinematics> kinematics_;

    // ROS 接口
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};