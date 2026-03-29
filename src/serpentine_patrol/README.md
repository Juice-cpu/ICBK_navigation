# Serpentine Patrol - S形路径导航功能包

## 功能描述

本功能包实现机器人从起始位置自动导航到终点位置，按照S形路径绕过4个转向标记的导航任务。

## 任务要求

- **路径**: 起始位置 → 绕过标记1 → 绕过标记2 → 绕过标记3 → 绕过标记4 → 终点位置
- **布局**: 水平距离6m，垂直距离1m（标记之间的间距）
- **约束**: 
  - 机器人不能与转向标记接触
  - 必须绕过每个转向标记
- **评分**: 从起始位置到终点位置的耗时

## 使用方法

### 1. 编译功能包

```bash
cd /home/jiang/pb_rm_simulation
colcon build --packages-select serpentine_patrol
source install/setup.bash
```

### 2. 启动导航系统

首先启动导航系统（仿真或实车模式）：

**仿真模式:**
```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py \
    world:=RMUL \
    mode:=nav \
    lio:=fastlio \
    localization:=slam_toolbox \
    lio_rviz:=False \
    nav_rviz:=True
```

**实车模式:**
```bash
ros2 launch rm_nav_bringup bringup_real.launch.py \
    world:=YOUR_WORLD_NAME \
    mode:=nav \
    lio:=fastlio \
    localization:=slam_toolbox \
    lio_rviz:=False \
    nav_rviz:=True
```

### 3. 启动S形导航节点

等待导航系统完全启动后（确保定位已初始化），在另一个终端运行：

```bash
ros2 launch serpentine_patrol serpentine_patrol.launch.py
```

或者直接运行节点：

```bash
ros2 run serpentine_patrol serpentine_patrol_node
```

## 调整路径点坐标

根据实际环境，您需要修改 `src/serpentine_patrol_node.cpp` 中的 `initializeWaypoints()` 函数来设置正确的路径点坐标。

### 步骤：

1. 在地图坐标系中确定：
   - 起始位置坐标
   - 4个转向标记的位置
   - 终点位置坐标
   - 绕过每个标记的路径点坐标

2. 修改 `src/serpentine_patrol_node.cpp` 中的坐标值：

```cpp
void initializeWaypoints()
{
    waypoints_.clear();
    
    // 修改这些坐标值以匹配您的实际环境
    // 点1: 起始位置
    start_pose.pose.position.x = 0.0;  // 修改为实际x坐标
    start_pose.pose.position.y = 0.0;  // 修改为实际y坐标
    
    // 点2-5: 绕过标记的路径点
    // ... 修改相应的坐标值
    
    // 点6: 终点位置
    end_pose.pose.position.x = 6.0;  // 修改为实际x坐标
    end_pose.pose.position.y = 4.0;  // 修改为实际y坐标
}
```

3. 重新编译：

```bash
colcon build --packages-select serpentine_patrol
source install/setup.bash
```

## 路径规划说明

- 使用 Navigation2 的 `navigate_through_poses` action，可以一次性规划通过所有路径点的路径
- 路径点会按顺序依次导航
- Navigation2 会自动进行路径规划和避障
- 确保在costmap中将转向标记标记为障碍物，机器人会自动绕过

## 时间记录

节点会在控制台输出：
- 导航开始时间
- 导航结束时间
- 总耗时（秒和毫秒）
- 评分时间（秒）

示例输出：
```
=== Navigation Started at: 2025-01-15 10:30:00 ===
=== Navigation Completed Successfully! ===
=== Navigation Ended at: 2025-01-15 10:30:45 ===
=== Total Time: 45.123 seconds (45123 milliseconds) ===
=== Score (Time): 45.123 seconds ===
```

## 注意事项

1. **坐标系统**: 确保使用的坐标系统与地图坐标系一致（通常是 `map` frame）
2. **定位初始化**: 启动导航前确保机器人已完成定位初始化
3. **路径点顺序**: 路径点必须按照导航顺序排列
4. **标记位置**: 在costmap中确保转向标记被正确标记为障碍物
5. **机器人尺寸**: 调整 Navigation2 参数中的 `robot_radius` 以确保机器人能够安全绕过标记

## 故障排查

- **Goal rejected**: 检查导航系统是否完全启动，定位是否初始化
- **路径规划失败**: 检查路径点坐标是否正确，是否在可达区域内
- **时间不准确**: 检查系统时钟是否同步

## 依赖

- ROS2 Humble
- Navigation2
- nav2_msgs
- geometry_msgs

