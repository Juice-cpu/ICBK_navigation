
#!/bin/bash

# 设置日志记录
exec > ~/ros2_autostart.log 2>&1
echo "===== 启动开始于 $(date) ====="

# 等待图形界面就绪（关键！）
while [ -z "$(pgrep gnome-session)" ]; do
    echo "等待GNOME会话启动..."
    sleep 2
done

# 加载环境变量（根据实际路径调整）
export DISPLAY=:0
export XAUTHORITY=/home/$USER/.Xauthority

source /opt/ros/humble/setup.bash
source ~/rm_navigation_ws/install/setup.bash

# 启动导航系统
echo "启动导航..."
gnome-terminal --title="Nav Sim" -- bash -c \
  "ros2 launch rm_nav_bringup bringup_real.launch.py \
    world:=RMUL \
    mode:=mapping \
    lio:=fastlio \
    lio_rviz:=False \
    nav_rviz:=False && exec bash"

# 动态等待节点启动（替代固定sleep）
echo "等待节点初始化..."
until ros2 node list | grep -q "/nav_bringup"; do
    sleep 2
done

# 启动速度转换
echo "启动速度转换..."
gnome-terminal --title="Vel Transform" -- bash -c \
  "ros2 launch real_vel_transform real_vel_transform.launch.py && exec bash"

# 等待话题可用
echo "等待/goal_pose话题..."
until ros2 topic list | grep -q "/goal_pose"; do
    sleep 1
done

# 发送目标点
echo "发送目标点..."
gnome-terminal --title="Goal Publisher" -- bash -c \
  "ros2 topic pub -1 /goal_pose geometry_msgs/msg/PoseStamped \
  '{ header: { frame_id: \"map\" }, pose: { position: { x: 2.0, y: -1.0, z: 0.0 }, orientation: { x: 0.0, y: 0.0, z: 0.0, w: 0.0 } } }' \
  && echo '目标点发送成功' || echo '发送失败'; exec bash"

echo "===== 启动完成于 $(date) ====="

