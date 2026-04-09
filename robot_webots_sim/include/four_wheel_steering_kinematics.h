#pragma once

#include <array>
#include <string>

#include "webots_driver.h"


inline double RAD2DEG(double rad) {return rad * 180.0 / M_PI;}
inline double DEG2RAD(double deg) {return deg * M_PI / 180.0;}

template <typename T>
inline int sign(T x) {
    if (x > 0) return 1;
    if (x < 0) return -1;
    return 1;
}

class FourWheelSteeringKinematics : public Kinematics {
public:
    FourWheelSteeringKinematics();
    void setVelocity(double vx, double vy, double omega) override;

private:
    // calculate target_steer_angle and targer_drive_angular_vel by input target vx,vy,omega
    void inverseKinematics(double vx, double vy, double omega, std::array<double, 4>& targer_steer, std::array<double, 4>& targer_drive);

    bool isSteerAlign(const std::array<double, 4>& target_steer);

    void setWheelCommands(std::array<double, 4> target_steer, std::array<double, 4> target_drive);

    // 你的四舵轮命名
    std::array<std::string, 4> STEER_MOTORS = {
        "steer_fl", "steer_fr", "steer_rl", "steer_rr"
    };

    std::array<std::string, 4> STEER_SENSORS = {
        "steer_fl_sensor", "steer_fr_sensor", "steer_rl_sensor", "steer_rr_sensor"
    };

    std::array<std::string, 4> DRIVE_MOTORS = {
        "drive_fl", "drive_fr", "drive_rl", "drive_rr"
    };

    std::array<std::string, 4> DRIVE_SENSORS = {
        "drive_fl_sensor", "drive_fr_sensor", "drive_rl_sensor", "drive_rr_sensor"
    };

    // 设备句柄
    std::array<WbDeviceTag, 4> steer_motors_;
    std::array<WbDeviceTag, 4> drive_motors_;
    std::array<WbDeviceTag, 4> steer_sensors_;
    std::array<WbDeviceTag, 4> drive_sensors_;

    // 四舵轮角度
    std::array<double, 4> steer_angles_;

    static constexpr double CHASSIS_HALF_WIDTH = 0.175;
    static constexpr double WHEEL_RADIUS = 0.05;
};