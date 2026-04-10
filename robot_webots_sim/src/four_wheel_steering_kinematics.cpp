#include "four_wheel_steering_kinematics.h"

FourWheelSteeringKinematics::FourWheelSteeringKinematics() {
    // 获取电机
    for (int i = 0; i < 4; i++) {
        steer_motors_[i] = wb_robot_get_device(STEER_MOTORS[i].c_str());
        drive_motors_[i] = wb_robot_get_device(DRIVE_MOTORS[i].c_str());
        steer_sensors_[i] = wb_robot_get_device(STEER_SENSORS[i].c_str());
        drive_sensors_[i] = wb_robot_get_device(DRIVE_SENSORS[i].c_str());
        
        wb_motor_set_position(steer_motors_[i], 0.0);
        wb_motor_set_position(drive_motors_[i], INFINITY);
        wb_motor_set_velocity(drive_motors_[i], 0.0);
        wb_position_sensor_enable(steer_sensors_[i], TIME_STEP);
        wb_position_sensor_enable(drive_sensors_[i], TIME_STEP);
    }
    RCLCPP_INFO(rclcpp::get_logger("FourWheelSteeringKinematics"), "Webots devices initialized.");
}


void FourWheelSteeringKinematics::setVelocity(double vx, double vy, double omega) {
    std::array<double, 4> target_steer, target_drive;
    inverseKinematics(vx, vy, omega, target_steer, target_drive);
    setWheelCommands(target_steer, target_drive);

    if (0) {
        RCLCPP_INFO(rclcpp::get_logger("FourWheelSteeringKinematics"),
            "vx: %.3f, vy: %.3f, omega: %.3f, "
            "target_steer: %.3f, %.3f, %.3f, %.3f, "
            "target_drive: %.3f, %.3f, %.3f, %.3f",
            vx, vy, omega,
            target_steer[0], target_steer[1], target_steer[2], target_steer[3],
            target_drive[0], target_drive[1], target_drive[2], target_drive[3]
        );
    }
}


void FourWheelSteeringKinematics::inverseKinematics(double vx, double vy, double omega,
        std::array<double, 4>& target_steer, std::array<double, 4>& target_drive) {
    
    double h = CHASSIS_HALF_WIDTH;
    double hw = h * omega;

    target_steer = {
        std::atan2((vy + hw), (vx - hw)),
        std::atan2((vy + hw), (vx + hw)),
        std::atan2((vy - hw), (vx - hw)),
        std::atan2((vy - hw), (vx + hw))
    };

    target_drive = {
        std::sqrt(std::pow(vx - hw, 2) + std::pow(hw + vy, 2)) / WHEEL_RADIUS,
        std::sqrt(std::pow(vx + hw, 2) + std::pow(hw + vy, 2)) / WHEEL_RADIUS,
        std::sqrt(std::pow(vx - hw, 2) + std::pow(hw - vy, 2)) / WHEEL_RADIUS,
        std::sqrt(std::pow(vx + hw, 2) + std::pow(hw - vy, 2)) / WHEEL_RADIUS
    };
}


bool FourWheelSteeringKinematics::isSteerAlign(const std::array<double, 4>& target_steer) {
    for (int i = 0; i < 4; ++i) {
        if (fabs(target_steer[i] - wb_position_sensor_get_value(steer_sensors_[i])) > DEG2RAD(5))
            return false;
    }
    return true;
}


void FourWheelSteeringKinematics::setWheelCommands(std::array<double, 4> target_steer, std::array<double, 4> target_drive) {
    // 这里将舵轮角度限制到[-pi/2, pi/2]范围内，防止舵轮需要大角度旋转
    for (size_t i = 0; i < 4; ++i) {
        if (target_steer[i] > M_PI / 2.0) {
            target_steer[i] -= M_PI;
            target_drive[i] *= -1.0;
        } else if (target_steer[i] <= -M_PI / 2.0) {
            target_steer[i] += M_PI;
            target_drive[i] *= -1.0;
        }
    }

    bool steering_align_done = isSteerAlign(target_steer);

    if (0){
        RCLCPP_INFO(rclcpp::get_logger("FourWheelSteeringKinematics"), 
            "steering_align_done: %d, target_steer: %.3f, %.3f, %.3f, %.3f, target_drive: %.3f, %.3f, %.3f, %.3f", 
            steering_align_done,
            target_steer[0], target_steer[1], target_steer[2], target_steer[3],
            target_drive[0], target_drive[1], target_drive[2], target_drive[3]
        );
    }

    for (size_t i = 0; i < 4; ++i) {
        wb_motor_set_position(steer_motors_[i], target_steer[i]);
        wb_motor_set_velocity(drive_motors_[i], steering_align_done ? target_drive[i] : 0.0);
    }
}

void FourWheelSteeringKinematics::initJointState(sensor_msgs::msg::JointState& joint_state_msg_) {
    joint_state_msg_.header.frame_id = "base_link";
    for (auto motor : STEER_MOTORS) joint_state_msg_.name.push_back(motor);
    for (auto drive : DRIVE_MOTORS) joint_state_msg_.name.push_back(drive);
    joint_state_msg_.position.resize(8, 0.0);
}


void FourWheelSteeringKinematics::updateJointState(sensor_msgs::msg::JointState& joint_state_msg_) {
    // 转向
    joint_state_msg_.position[0] = wb_position_sensor_get_value(steer_sensors_[0]);
    joint_state_msg_.position[1] = wb_position_sensor_get_value(steer_sensors_[1]);
    joint_state_msg_.position[2] = wb_position_sensor_get_value(steer_sensors_[2]);
    joint_state_msg_.position[3] = wb_position_sensor_get_value(steer_sensors_[3]);

    // 驱动
    joint_state_msg_.position[4] = wb_position_sensor_get_value(drive_sensors_[0]);
    joint_state_msg_.position[5] = wb_position_sensor_get_value(drive_sensors_[1]);
    joint_state_msg_.position[6] = wb_position_sensor_get_value(drive_sensors_[2]);
    joint_state_msg_.position[7] = wb_position_sensor_get_value(drive_sensors_[3]);
}


void FourWheelSteeringKinematics::forwardKinematics (
    const std::array<double, 4>& steer,    // 输入：4个轮子的转向角
    const std::array<double, 4>& drive,    // 输入：4个驱动轮的角速度
    double& Vx, double& Vy, double& Omega) // 输出：底盘速度
{
    const double h = CHASSIS_HALF_WIDTH;
    const double r = WHEEL_RADIUS;

    // 步骤1：计算每个轮子的速度分量 (vx_i, vy_i)
    std::array<double, 4> vx, vy;
    for (int i = 0; i < 4; ++i) {
        double v_wheel = drive[i] * r;               // 轮子线速度
        vx[i] = v_wheel * std::cos(steer[i]);        // x方向分量
        vy[i] = v_wheel * std::sin(steer[i]);        // y方向分量
    }

    // 步骤2：最小二乘求解底盘速度
    Vx = (vx[0] + vx[1] + vx[2] + vx[3]) / 4.0;
    Vy = (vy[0] + vy[1] + vy[2] + vy[3]) / 4.0;

    double term = -vx[0] + vy[0] + vx[1] + vy[1] - vx[2] - vy[2] + vx[3] - vy[3];
    Omega = term / (8.0 * h);
}

void FourWheelSteeringKinematics::updateOdometryTwist(geometry_msgs::msg::Twist& twist) {
    std::array<double, 4> cur_steer_angle, cur_drive_vel;
    for (size_t i = 0; i < 4; ++i) {
        cur_steer_angle[i] = wb_position_sensor_get_value(steer_sensors_[i]);
        cur_drive_vel[i] = wb_motor_get_velocity(drive_motors_[i]);
    }

    forwardKinematics(cur_steer_angle, cur_drive_vel, twist.linear.x, twist.linear.y, twist.angular.z);
}
