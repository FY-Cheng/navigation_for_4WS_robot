#include "webots_driver.h"
#include "four_wheel_steering_kinematics.h"

WebotsDriver::WebotsDriver() : Node("webots_driver") {
    // 启用仿真时间
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));

    // 连接 Webots
    wb_robot_init();
    RCLCPP_INFO(this->get_logger(), "Webots extern 控制器已连接");

    // 初始化传感器
    imu_ = wb_robot_get_device("inertial unit");
    gps_ = wb_robot_get_device("gps");
    wb_inertial_unit_enable(imu_, TIME_STEP);
    wb_gps_enable(gps_, TIME_STEP);

    // ==============================
    // 运动学解耦：这里替换成你的底盘
    // ==============================
    kinematics_ = std::make_unique<FourWheelSteeringKinematics>();
    kinematics_->initJointState(joint_state_msg_);

    // ROS 发布者
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

    // 订阅速度
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&WebotsDriver::cmdVelCallback, this, std::placeholders::_1)
    );

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // 主循环
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(TIME_STEP),
        std::bind(&WebotsDriver::updateLoop, this)
    );
}

WebotsDriver::~WebotsDriver() {
    wb_robot_cleanup();
}

void WebotsDriver::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    kinematics_->setVelocity(msg->linear.x, msg->linear.y, msg->angular.z);
}

void WebotsDriver::updateLoop() {
    if (wb_robot_step(TIME_STEP) == -1) {
        rclcpp::shutdown();
        return;
    }
    
    publishIMU();
    publishOdometryAndJointState();
}

void WebotsDriver::publishIMU() {
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.frame_id = "imu_link";
    imu_msg.header.stamp = this->now();

    const double* q = wb_inertial_unit_get_quaternion(imu_);
    imu_msg.orientation.w = q[3];
    imu_msg.orientation.x = q[0];
    imu_msg.orientation.y = q[1];
    imu_msg.orientation.z = q[2];

    imu_pub_->publish(imu_msg);
}

void WebotsDriver::publishOdometryAndJointState() {
    rclcpp::Time now = this->now();

    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    odom.header.stamp = now;
    const double* pos = wb_gps_get_values(gps_);
    const double* q = wb_inertial_unit_get_quaternion(imu_);
    odom.pose.pose.position.x = pos[0];
    odom.pose.pose.position.y = pos[1];
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.w = q[3];
    odom.pose.pose.orientation.x = q[0];
    odom.pose.pose.orientation.y = q[1];
    odom.pose.pose.orientation.z = q[2];
    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped tf;
    tf.header = odom.header;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_footprint";
    tf.transform.translation.x = pos[0];
    tf.transform.translation.y = pos[1];
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);

    kinematics_->updateJointState(joint_state_msg_);
    joint_state_msg_.header.stamp = now;
    joint_state_pub_->publish(joint_state_msg_);
}


// 发布仿真时钟
void WebotsDriver::publishClock() {
    rosgraph_msgs::msg::Clock clock_msg;
    clock_msg.clock.sec = wb_robot_get_time();
    clock_msg.clock.nanosec = (wb_robot_get_time() - (int)wb_robot_get_time()) * 1e9;
    clock_pub_->publish(clock_msg);
}


void WebotsDriver::run() {
    while (wb_robot_step(TIME_STEP) != -1) {
        // 1. 发布仿真时钟 → 时间戳正常
        publishClock();
        // 2. 发布传感器+里程计+TF
        publishIMU();
        publishOdometryAndJointState();
        // 3. 非阻塞处理ROS2回调（cmd_vel）
        rclcpp::spin_some(this->shared_from_this());
    }
}


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WebotsDriver>();
    node->run();  // 启动Webots主循环
    rclcpp::shutdown();
    return 0;
}