#include "webots_driver.h"
#include "four_wheel_steering_kinematics.h"

WebotsDriver::WebotsDriver() : Node("webots_driver") {
    // 启用仿真时间
    // this->declare_parameter("use_sim_time", true);
    // this->set_parameter(rclcpp::Parameter("use_sim_time", true));

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

    // ROS 发布者
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);

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
    double vx = msg->linear.x;
    double vy = msg->linear.y;
    double omega = msg->angular.z;
    kinematics_->setVelocity(vx, vy, omega);
}

void WebotsDriver::updateLoop() {
    if (wb_robot_step(TIME_STEP) == -1) {
        rclcpp::shutdown();
        return;
    }
    
    publishIMU();
    publishOdometry();
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

void WebotsDriver::publishOdometry() {
    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    odom.header.stamp = this->now();

    const double* pos = wb_gps_get_values(gps_);
    odom.pose.pose.position.x = pos[0];
    odom.pose.pose.position.y = pos[1];

    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped tf;
    tf.header = odom.header;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_footprint";
    tf.transform.translation.x = pos[0];
    tf.transform.translation.y = pos[1];
    tf.transform.rotation = odom.pose.pose.orientation;

    tf_broadcaster_->sendTransform(tf);
}




int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WebotsDriver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}