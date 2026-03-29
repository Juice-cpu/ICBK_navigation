# armor_chaser

基于 Nav2 的动态目标追击节点。订阅装甲板跟踪结果，按目标朝向后侧跟随并持续向 Nav2 下发更新的 `NavigateToPose` 目标，可选使用全局 costmap 在目标周围选择低成本“攻击点”，优先目标朝向前半圆。

## 主要节点
- `chase_state_machine` (`armor_chaser/chase_state_machine_node`)

## 订阅话题
- `tracker/target` (`rm_interfaces/msg/Target`): 来自 armor_tracker_node 的目标位姿、速度、朝向和 tracking 标志。
- `global_costmap/costmap` (`nav_msgs/msg/OccupancyGrid`, transient_local): 可选，用于攻击点筛选。

## 发布 / 动作
- Nav2 Action `navigate_to_pose` (`nav2_msgs/action/NavigateToPose`): 持续更新的追击目标。

## 关键参数
- `update_distance_threshold` (double, default 0.5): 新旧 goal 距离超过该值才刷新，避免抖动。
- `replan_period` (double, default 0.3): 即使距离未超阈值，多少秒强制刷新一次。
- `lost_timeout` (double, default 1.0): 超出该时间未收目标判定彻底丢失并取消目标。
- `lost_grace` (double, default 0.4): 短时丢失宽限，继续追最后预测位置。
- `goal_frame` (string, default `map`): 目标坐标系。
- `goal_offset_x`, `goal_offset_y` (double, default 0): 目标平移偏置。
- `follow_distance` (double, default 0.5): 沿目标朝向后退的跟随距离。
- `prediction_horizon` (double, default 0.2): 速度前视时间。
- `smoothing_alpha` (double, default 0.7): 位置低通系数，越大越贴合。
- `attack_radius` (double, default 1.5): 攻击点采样半径。
- `attack_num_sectors` (int, default 12): 攻击点圆周采样数。
- `attack_cost_threshold` (int, default 50): 可接受的最大 cost（0–100）。

## 行为逻辑
- 状态机：`SEARCHING` → `CHASING`（tracking=true），短时丢失进入 `TEMP_LOST`，超宽限回 `SEARCHING`，超 `lost_timeout` 进入 `LOST` 并取消目标。
- 目标生成：位置+速度前视 → 低通平滑 → 沿目标朝向后方 `follow_distance` 与偏置 → 朝向指向目标位置。
- 攻击点选择：若有 costmap，在目标周围半径 `attack_radius` 圆环采样 `attack_num_sectors` 点，优先选择目标航向前半圆（±90°）内且 cost 低于 `attack_cost_threshold` 的最低成本点；若前半圆无合格点，退回全圆最低成本；否则保持原位。
- 刷新策略：距离超 `update_distance_threshold` 或时间超 `replan_period` 才发送新 goal。

## 启动示例
```bash
ros2 launch armor_chaser chase_state_machine.launch.py \
  update_distance_threshold:=0.5 \
  lost_timeout:=1.0 \
  goal_frame:=map \
  goal_offset_x:=0.0 \
  goal_offset_y:=0.0
```

确保 Nav2 `navigate_to_pose` action server 与 `tracker/target`、`global_costmap/costmap` 话题可用。根据场地调整 `attack_radius` / `attack_cost_threshold` / `follow_distance` 等参数。
