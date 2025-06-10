# EGO-Planner SLAM适配启动文件使用指南

## 📖 概述

`single_run_in_sim_slam_adapted.launch.py` 是专门为IR100 SLAM系统适配的EGO-Planner启动文件。该文件支持多种运行模式，包括真实机器人、仿真环境以及回放数据等场景。

## 🔧 新增参数说明

### 仿真时间参数 (use_sim_time)

| 参数名 | 类型 | 默认值 | 描述 |
|--------|------|--------|------|
| `use_sim_time` | bool | false | 是否使用仿真时间同步 |

**使用场景：**
- **true**: 仿真环境、rosbag回放、Gazebo仿真
- **false**: 真实机器人、实时SLAM建图

## 🚀 使用方法

### 1. 真实机器人模式 (推荐配置)

```bash
# 使用FastLIO2里程计的真实机器人模式
ros2 launch plan_manage single_run_in_sim_slam_adapted.launch.py \
    use_simulation_map:=false \
    use_dynamic_simulation:=false \
    use_sim_time:=false \
    odom_topic:=/fastlio2/lio_odom

# 使用定位器的真实机器人模式
ros2 launch plan_manage single_run_in_sim_slam_adapted.launch.py \
    use_simulation_map:=false \
    use_localization:=true \
    use_sim_time:=false \
    odom_topic:=/localization/odom
```

### 2. 仿真环境模式

```bash
# 完整仿真模式 (包含地图生成器和动力学仿真)
ros2 launch plan_manage single_run_in_sim_slam_adapted.launch.py \
    use_simulation_map:=true \
    use_dynamic_simulation:=true \
    use_sim_time:=true \
    odom_topic:=/odom

# 混合仿真模式 (使用真实SLAM + 仿真规划)
ros2 launch plan_manage single_run_in_sim_slam_adapted.launch.py \
    use_simulation_map:=false \
    use_dynamic_simulation:=true \
    use_sim_time:=true \
    odom_topic:=/fastlio2/lio_odom
```

### 3. 数据回放模式

```bash
# Step 1: 启动EGO-Planner (使用仿真时间)
ros2 launch plan_manage single_run_in_sim_slam_adapted.launch.py \
    use_simulation_map:=false \
    use_dynamic_simulation:=false \
    use_sim_time:=true \
    odom_topic:=/fastlio2/lio_odom

# Step 2: 在另一个终端回放rosbag数据
ros2 bag play your_slam_data.bag --clock
```

### 4. 调试模式

```bash
# 启用SLAM状态监控的调试模式
ros2 launch plan_manage single_run_in_sim_slam_adapted.launch.py \
    use_simulation_map:=false \
    use_sim_time:=false \
    odom_topic:=/fastlio2/lio_odom \
    --ros-args --log-level DEBUG
```

## ⚙️ 参数组合表

| 使用场景 | use_simulation_map | use_dynamic_simulation | use_sim_time | use_localization |
|----------|-------------------|----------------------|--------------|------------------|
| **真实机器人 + FastLIO2** | false | false | false | false |
| **真实机器人 + Localizer** | false | false | false | true |
| **完整仿真环境** | true | true | true | false |
| **SLAM数据回放** | false | false | true | false |
| **仿真测试SLAM** | false | true | true | false |
| **离线地图规划** | false | false | false | false |

## 🔗 系统集成启动序列

### 真实系统启动序列

```bash
# 1. 启动IR100 SLAM核心服务
ros2 launch fastlio2 fastlio2.launch.py use_sim_time:=false
ros2 launch pgo pgo.launch.py use_sim_time:=false
ros2 launch localizer localizer_launch.py use_sim_time:=false

# 2. 启动导航服务 (可选)
ros2 launch navigation cloud_to_2d_converter.launch.py use_sim_time:=false
ros2 launch navigation nav2_bringup.launch.py use_sim_time:=false

# 3. 启动EGO-Planner
ros2 launch plan_manage single_run_in_sim_slam_adapted.launch.py \
    use_sim_time:=false \
    odom_topic:=/fastlio2/lio_odom
```

### 仿真系统启动序列

```bash
# 1. 启动仿真环境 (如Gazebo)
ros2 launch gazebo_ros gazebo.launch.py use_sim_time:=true

# 2. 启动机器人仿真
ros2 launch robot_simulation robot.launch.py use_sim_time:=true

# 3. 启动SLAM (如果需要)
ros2 launch fastlio2 fastlio2.launch.py use_sim_time:=true

# 4. 启动EGO-Planner
ros2 launch plan_manage single_run_in_sim_slam_adapted.launch.py \
    use_simulation_map:=true \
    use_dynamic_simulation:=true \
    use_sim_time:=true
```

### 数据回放启动序列

```bash
# 1. 设置ROS参数
ros2 param set /use_sim_time true

# 2. 启动EGO-Planner
ros2 launch plan_manage single_run_in_sim_slam_adapted.launch.py \
    use_sim_time:=true \
    odom_topic:=/fastlio2/lio_odom

# 3. 回放数据 (包含时钟信息)
ros2 bag play slam_data.bag --clock 100
```

## ⚠️ 重要注意事项

### 时间同步要求

1. **所有节点必须使用相同的时间源**
   - 仿真模式：所有节点使用 `use_sim_time:=true`
   - 真实模式：所有节点使用 `use_sim_time:=false`

2. **时钟发布要求**
   - 仿真模式需要 `/clock` 话题发布仿真时间
   - Gazebo、rosbag play --clock 会自动发布时钟
   - **注意**: rosbag回放时使用 `--clock` 参数会自动提供时钟，无需额外时钟发布器

3. **数据时间戳一致性**
   - 确保SLAM数据和规划数据使用相同时间戳
   - 检查TF数据的时间有效性

### 话题同步检查

```bash
# 检查时间同步状态
ros2 param get /use_sim_time
ros2 topic echo /clock --once

# 检查关键话题的时间戳
ros2 topic echo /fastlio2/lio_odom --field header.stamp
ros2 topic echo /fastlio2/body_cloud --field header.stamp

# 检查TF树时间
ros2 run tf2_ros tf2_echo map base_link
```

### 故障排除

#### 时间同步问题
```bash
# 问题: 节点时间不同步
# 解决: 重启所有节点并确保use_sim_time参数一致

# 检查参数设置
ros2 param list | grep use_sim_time
ros2 param get /your_node use_sim_time
```

#### 话题延迟问题
```bash
# 检查话题延迟
ros2 topic hz /fastlio2/lio_odom
ros2 topic delay /fastlio2/lio_odom

# 检查系统负载
htop
ros2 node list
```

## 📊 性能监控

### 实时性能监控

```bash
# 监控关键话题频率
ros2 run rqt_topic rqt_topic

# 监控计算资源
ros2 run rqt_top rqt_top

# 监控网络流量
ros2 run rqt_graph rqt_graph
```

### 日志分析

```bash
# 查看节点日志
ros2 log info /your_node_name

# 保存日志到文件
ros2 launch plan_manage single_run_in_sim_slam_adapted.launch.py \
    --ros-args --log-level DEBUG > ego_planner.log 2>&1
```

## 🎯 最佳实践

1. **参数一致性**: 确保整个系统的 `use_sim_time` 参数保持一致
2. **渐进式启动**: 按照推荐的启动序列逐步启动各个模块
3. **状态监控**: 使用提供的监控工具检查系统状态
4. **日志记录**: 保存重要的运行日志用于问题分析
5. **性能调优**: 根据硬件配置调整仿真频率和处理参数

---

*更新时间: 2025-01-09*  
*版本: v1.0.0*  
*维护者: IR100 SLAM项目组* 