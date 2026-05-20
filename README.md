# 四舵轮机器人导航系统 (navigation_for_4WS_robot)

本项目实现了一个基于ROS2和Nav2的四舵轮机器人导航系统，包含Webots仿真、SLAM建图、路径规划和运动控制等功能。

## 主要功能

- 四舵轮运动学模型实现
- 2D建图(SLAM Toolbox)
- Nav2导航栈集成
- Webots仿真环境支持
- 机器人状态发布和TF配置

## 快速开始

### 启动仿真环境+slam定位
```bash
ros2 launch robot_bringup webots_bringup.launch.py 
```

### 启动导航
```bash
ros2 launch robot_navigation navigation2.launch.py
```

## 模块说明

- `robot_bringup`: 机器人启动配置和launch文件
- `robot_slam_toolbox`: SLAM工具箱配置
- `robot_description`: 机器人URDF模型
- `robot_navigation`: Nav2导航配置
- `robot_webots_sim`: Webots仿真环境

## 许可证

本项目使用 [MIT License](LICENSE)
