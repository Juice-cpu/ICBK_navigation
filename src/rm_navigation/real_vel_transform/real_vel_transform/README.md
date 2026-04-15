
realveltransform
real_vel_transform 是一个用于将机器人速度命令从 base_link 坐标系转换到 base_link_fake 坐标系的 ROS 2 功能包。该包会定期从 /cmd_vel 订阅速度命令，使用 TF 变换将其从 base_link 坐标系转换到新的坐标系，然后发布转换后的速度命令到 /cmd_vel_chassis 话题。同时，转换后的速度命令会通过串口发送到控制器（如 STM32）。
功能

1.从 /cmd_vel 订阅速度命令（geometry_msgs::msg::Twist）。
2.使用 TF 获取当前机器人的姿态（从 odom 到 base_link 坐标系的变换）。
3.将接收到的速度命令根据机器人的姿态进行坐标转换，得到新的速度命令。
4.将转换后的速度命令发布到 /cmd_vel_chassis 话题。
5.通过串口发送转换后的速度命令给外部控制器（如 STM32）。
6.定期发布 base_link 到 base_link_fake 的 TF 变换。

依赖

ROS 2（适用于 ROS 2 Foxy、Galactic 或 Humble）
serial 库（用于串口通信）
tf2 库（用于 TF 变换处理）
geometry_msgs 和 nav_msgs（用于消息处理）

安装

确保您的系统已经安装 ROS 2（例如，使用 ROS 2 安装教程）。
将该功能包克隆到您的工作空间：

   cd ~/ros2_ws/src
   git clone &lt;your-repository-url&gt;/real_vel_transform.git


安装依赖：

   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y


编译工作空间：

   colcon build --symlink-install


安装串口库依赖：
如果您还没有安装 serial 库，您需要安装它：

   sudo apt-get install libserial-dev

配置
在运行功能包之前，您可以根据需要调整以下参数：

spin_speed: 控制机器人旋转时的速度（单位：m/s）。默认值为 -6.0。

可以通过 ROS 参数服务器来设置参数，或者在启动文件中指定。例如：
spin_speed: -8.0

使用方法

启动 real_vel_transform 功能包：
您可以通过 ROS 2 启动文件或者命令行启动功能包。
使用以下命令启动节点：

   ros2 run real_vel_transform real_vel_transform_node


发布速度命令到 /cmd_vel 话题：
您可以使用 ROS 2 的 teleop 工具发布速度命令，或者您可以手动发布 geometry_msgs::msg::Twist 消息到 /cmd_vel 话题。
例如，使用 ros2 topic pub 命令手动发布速度命令：

   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}"

这将使机器人以 1.0 m/s 的线速度和 0.5 rad/s 的角速度移动。

查看转换后的速度命令：
您可以通过 ros2 topic echo 命令查看 /cmd_vel_chassis 话题上的转换后的速度命令：

   ros2 topic echo /cmd_vel_chassis


串口通信：
功能包会自动通过串口将转换后的速度命令发送到控制器（例如 STM32）。您需要确保串口设备 /dev/myserial 正确连接并且已打开。

重要话题

/cmd_vel: 接收来自外部系统的速度命令（类型：geometry_msgs::msg::Twist）。
/cmd_vel_chassis: 发布转换后的速度命令（类型：geometry_msgs::msg::Twist）。
/local_plan: 发布机器人规划的路径（类型：nav_msgs::msg::Path），用于计算机器人的当前姿态。

TF 变换
该功能包会定期发布一个 TF 变换，从 base_link 到 base_link_fake，用于显示机器人当前的姿态。您可以使用以下命令查看 TF 树：
ros2 run tf2_tools view_frames

这将生成一个 frames.pdf 文件，展示机器人各个坐标系的关系。
错误处理与日志

如果串口无法打开，系统将发出警告信息。
如果无法从 TF 中查找变换，会在日志中记录错误信息。
如果 TF 变换失败或其他异常，系统会记录详细的异常信息。

贡献
欢迎提交 Pull Requests 和 Issues。如果您有任何问题或建议，欢迎随时提出。
授权
该功能包基于 MIT 许可证。

其他备注：

依赖项：这个 README 文件假设您已经安装了 ROS 2 的基础依赖项，如果没有，请根据 ROS 2 的版本文档进行安装。
串口连接：请根据您的硬件调整串口设备名称 /dev/myserial，确保系统能够访问正确的设备。
TF 验证：为了确保 TF 发布正常工作，您可以使用 ros2 run tf2_tools view_frames 来检查所有相关的 TF 帧。

