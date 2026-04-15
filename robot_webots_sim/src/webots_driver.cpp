#include "webots_driver.h"
#include "four_wheel_steering_kinematics.h"

WebotsDriver::WebotsDriver() : Node("webots_driver") {
    // 启用仿真时间
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));

    // 连接 Webots
    wb_robot_init();
    RCLCPP_INFO(this->get_logger(), "Webots extern 控制器已连接");

    // 初始化传感器
    imu_ = wb_robot_get_device("imu");
    gyro_ = wb_robot_get_device("gyro");
    accelerometer_ = wb_robot_get_device("accelerometer");
    gps_ = wb_robot_get_device("gps");
    lidar_3d_ = wb_robot_get_device("lidar");
    tof_ = wb_robot_get_device("tof");
    wb_inertial_unit_enable(imu_, TIME_STEP);
    wb_gyro_enable(gyro_, TIME_STEP);
    wb_accelerometer_enable(accelerometer_, TIME_STEP);
    wb_gps_enable(gps_, TIME_STEP);
    wb_lidar_enable(lidar_3d_, TIME_STEP);
    wb_range_finder_enable(tof_, TIME_STEP);
    wb_lidar_enable_point_cloud(lidar_3d_);

    // ==============================
    // 运动学解耦：这里替换成你的底盘
    // ==============================
    kinematics_ = std::make_unique<FourWheelSteeringKinematics>();
    kinematics_->initJointState(joint_state_msg_);

    // ROS 发布者
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
    lidar_3d_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar/point_cloud", 10);
    tof_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/tof/depth/image_raw", 10);
    tof_camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/tof/depth/camera_info", 10);
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
    imu_msg.header.frame_id = "imu";
    imu_msg.header.stamp = this->now();

    const double* q = wb_inertial_unit_get_quaternion(imu_);
    imu_msg.orientation.w = q[3];
    imu_msg.orientation.x = q[0];
    imu_msg.orientation.y = q[1];
    imu_msg.orientation.z = q[2];

    const double* angular_vel = wb_gyro_get_values(gyro_);
    imu_msg.angular_velocity.x = angular_vel[0];
    imu_msg.angular_velocity.y = angular_vel[1];
    imu_msg.angular_velocity.z = angular_vel[2];

    const double* linear_acc = wb_accelerometer_get_values(accelerometer_);
    imu_msg.linear_acceleration.x = linear_acc[0];
    imu_msg.linear_acceleration.y = linear_acc[1];
    imu_msg.linear_acceleration.z = linear_acc[2];

    // 姿态协方差
    std::fill(imu_msg.orientation_covariance.begin(), imu_msg.orientation_covariance.end(), 0.01);
    // 陀螺仪协方差
    std::fill(imu_msg.angular_velocity_covariance.begin(), imu_msg.angular_velocity_covariance.end(), 0.01);
    // 加速度计协方差
    std::fill(imu_msg.linear_acceleration_covariance.begin(), imu_msg.linear_acceleration_covariance.end(), 0.01);

    imu_pub_->publish(imu_msg);
}

void WebotsDriver::publishTof() {
    rclcpp::Time now = this->now();
    const float* wb = wb_range_finder_get_range_image(tof_);
    int w = wb_range_finder_get_width(tof_);
    int h = wb_range_finder_get_height(tof_);

    if (!wb || w <=0 || h <=0) return;

    sensor_msgs::msg::Image img;
    img.header.stamp = now;
    img.header.frame_id = "tof_optical";   
    img.width = w;
    img.height = h;
    img.encoding = "32FC1";
    img.step = w * 4;
    img.data.resize(img.step * h);

    memcpy(img.data.data(), wb, img.data.size());

    tof_image_pub_->publish(img);

    // 相机内参完全不变
    sensor_msgs::msg::CameraInfo info;
    info.header = img.header;
    info.width = w;
    info.height = h;
    info.distortion_model = "plumb_bob";
    info.d.resize(5, 0.0);

    double fov = 1.7104;
    double fx = (w / 2.0) / tan(fov / 2.0);
    double fy = fx;
    double cx = w / 2.0;
    double cy = h / 2.0;

    info.k = {fx, 0, cx, 0, fy, cy, 0, 0, 1};
    info.p = {fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0};

    tof_camera_info_pub_->publish(info);
}

// 纯转换：Webots 3D激光雷达点云 → ROS2标准PointCloud2，无多余操作，无编译错误
void WebotsDriver::publish3DPoints() {
    rclcpp::Time now = this->now();
    // 获取Webots原生点云
    const WbLidarPoint* webots_points = wb_lidar_get_point_cloud(lidar_3d_);
    int total_points = wb_lidar_get_number_of_points(lidar_3d_);

    // 仅空值检查（唯一必要判断）
    if (!webots_points || total_points <= 0) {
        return;
    }

    // 初始化ROS2标准PointCloud2
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = now;
    cloud.header.frame_id = "lidar";  // 标准坐标系

    // 激光雷达无序点云
    cloud.height = 1;
    cloud.width = total_points;
    cloud.is_bigendian = false;
    cloud.is_dense = false;

    cloud.point_step = 16;
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.data.resize(cloud.row_step);

    // 定义标准xyz字段（Webots仅支持这三个）
    cloud.fields.resize(4);
    // X
    cloud.fields[0].name = "x";
    cloud.fields[0].offset = 0;
    cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[0].count = 1;
    // Y
    cloud.fields[1].name = "y";
    cloud.fields[1].offset = 4;
    cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[1].count = 1;
    // Z
    cloud.fields[2].name = "z";
    cloud.fields[2].offset = 8;
    cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[2].count = 1;

    cloud.fields[3].name = "intensity";
    cloud.fields[3].offset = 12;
    cloud.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[3].count = 1;

    // 直接填充数据（纯复制，无修改、无过滤、无旋转）
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud, "intensity");


    for (int i = 0; i < total_points; ++i) {
        const auto& p = webots_points[i];
        *iter_x = static_cast<float>(p.x);
        *iter_y = static_cast<float>(p.y);
        *iter_z = static_cast<float>(p.z);
        *iter_intensity = 100.0f;  // 固定合理强度值

        ++iter_x;
        ++iter_y;
        ++iter_z;
        ++iter_intensity;
    }

    // 发布ROS2标准点云
    lidar_3d_pub_->publish(cloud);
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
    kinematics_->updateOdometryTwist(odom.twist.twist);
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

    joint_state_msg_.header.stamp = now;
    kinematics_->updateJointState(joint_state_msg_);
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
        publish3DPoints();
        publishTof();
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