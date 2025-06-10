# 规划管理器 (plan_manage / ego_planner)

## 概述

`plan_manage` (包名为 `ego_planner`) 是EGO-Planner-Swarm系统的核心规划管理模块，实现了基于有限状态机(FSM)的轨迹规划与重规划逻辑。该模块集成了路径搜索、轨迹优化和多机器人协调功能，提供了完整的自主导航解决方案。

## 主要功能

### 1. 有限状态机管理
- **INIT**: 系统初始化状态
- **WAIT_TARGET**: 等待目标点状态
- **GEN_NEW_TRAJ**: 生成新轨迹状态
- **REPLAN_TRAJ**: 轨迹重规划状态
- **EXEC_TRAJ**: 执行轨迹状态
- **EMERGENCY_STOP**: 紧急停止状态
- **SEQUENTIAL_START**: 顺序启动状态

### 2. 轨迹规划策略
- **全局路径规划**: 基于A*算法的初始路径搜索
- **局部轨迹优化**: B样条轨迹优化
- **动态重规划**: 基于环境变化的实时重规划
- **多机器人协调**: 避免机器人间碰撞的轨迹调整

### 3. 目标管理
- **手动目标**: 通过RViz交互式设置目标
- **预设目标**: 硬编码的路径点序列
- **参考路径**: 基于全局路径的目标生成

## 核心组件

### EGOReplanFSM类

```cpp
class EGOReplanFSM {
private:
    // 状态机状态枚举
    enum FSM_EXEC_STATE {
        INIT,           // 初始化
        WAIT_TARGET,    // 等待目标
        GEN_NEW_TRAJ,   // 生成新轨迹
        REPLAN_TRAJ,    // 重规划轨迹
        EXEC_TRAJ,      // 执行轨迹
        EMERGENCY_STOP, // 紧急停止
        SEQUENTIAL_START // 顺序启动
    };
    
    // 目标类型枚举
    enum TARGET_TYPE {
        MANUAL_TARGET = 1,  // 手动目标
        PRESET_TARGET = 2,  // 预设目标
        REFENCE_PATH = 3    // 参考路径
    };
};
```

### EGOPlannerManager类

```cpp
class EGOPlannerManager {
public:
    // 路径搜索和轨迹优化
    bool kinodynamicReplan(const Eigen::Vector3d& start_pt,
                          const Eigen::Vector3d& start_vel,
                          const Eigen::Vector3d& start_acc,
                          const Eigen::Vector3d& end_pt,
                          const Eigen::Vector3d& end_vel);
    
    // 应急停止轨迹生成
    bool emergencyStop(const Eigen::Vector3d& stop_pos);
    
    // 多机器人轨迹管理
    void setSwarmTrajs(traj_utils::msg::MultiBsplines::ConstPtr msg);
    bool checkCollisionWithSurroundTrajs();
};
```

## 状态机逻辑

### 状态转换图
```
    INIT
      ↓
  WAIT_TARGET ←─────────┐
      ↓                │
  GEN_NEW_TRAJ         │
      ↓                │
   EXEC_TRAJ ←─────── REPLAN_TRAJ
      ↓                ↑
  EMERGENCY_STOP ──────┘
```

### 关键状态处理

#### 1. WAIT_TARGET状态
- 等待目标点输入
- 检查目标点有效性
- 准备轨迹规划初始条件

#### 2. GEN_NEW_TRAJ状态
- 调用路径搜索算法
- 执行B样条轨迹优化
- 检查轨迹可行性

#### 3. EXEC_TRAJ状态
- 发布轨迹给控制器
- 监控轨迹执行状态
- 检测重规划触发条件

#### 4. REPLAN_TRAJ状态
- 基于当前状态重新规划
- 保持轨迹连续性
- 处理动态障碍物

#### 5. EMERGENCY_STOP状态
- 生成安全停止轨迹
- 广播紧急停止信号
- 等待安全确认

## 规划流程

### 1. 前端路径搜索
```cpp
bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj) {
    // 1. 调用A*搜索算法
    bool search_success = planner_manager_->kinodynamicReplan(
        start_pt_, start_vel_, start_acc_, 
        end_pt_, end_vel_);
    
    // 2. 检查搜索结果
    if (!search_success) {
        return false;
    }
    
    // 3. 轨迹后处理
    return true;
}
```

### 2. 后端轨迹优化
- **B样条参数化**: 将路径转换为B样条表示
- **梯度优化**: 基于梯度下降的轨迹优化
- **约束处理**: 动力学约束和碰撞约束
- **多机器人协调**: 考虑其他机器人轨迹的约束

### 3. 碰撞检测
```cpp
bool checkCollision() {
    // 1. 检查与静态障碍物碰撞
    bool static_collision = planner_manager_->checkTrajCollision();
    
    // 2. 检查与其他机器人轨迹碰撞
    bool swarm_collision = planner_manager_->checkCollisionWithSurroundTrajs();
    
    return static_collision || swarm_collision;
}
```

## 多机器人协调

### 1. 轨迹同步
- 接收其他机器人的轨迹信息
- 时间同步和空间冲突检测
- 优先级机制处理冲突

### 2. 通信接口
```cpp
// 订阅其他机器人轨迹
rclcpp::Subscription<traj_utils::msg::MultiBsplines>::SharedPtr swarm_trajs_sub_;

// 发布自己的轨迹
rclcpp::Publisher<traj_utils::msg::Bspline>::SharedPtr bspline_pub_;
```

### 3. 冲突解决策略
- **时间调整**: 调整轨迹时间参数避免冲突
- **路径重规划**: 重新搜索无冲突路径
- **优先级排队**: 基于机器人ID的优先级机制

## 配置参数

### 规划参数
```yaml
# 重规划阈值
no_replan_thresh: 2.0      # 不需要重规划的距离阈值
replan_thresh: 1.0         # 触发重规划的距离阈值

# 规划范围
planning_horizen: 7.5      # 规划距离范围
planning_horizen_time: 3.0 # 规划时间范围

# 安全参数
emergency_time: 1.0        # 紧急停止时间
enable_fail_safe: true     # 启用故障安全机制

# 实验模式
flag_realworld_experiment: false # 真实世界实验标志
```

### 路径点配置
```yaml
# 预设路径点 (最多50个)
waypoints:
  - [0.0, 0.0, 1.0]
  - [5.0, 0.0, 1.0] 
  - [5.0, 5.0, 1.0]
  - [0.0, 5.0, 1.0]
```

## ROS2接口

### 订阅话题
| 话题名 | 消息类型 | 描述 |
|--------|----------|------|
| `/waypoint` | geometry_msgs/PoseStamped | 手动设置的目标点 |
| `/odometry` | nav_msgs/Odometry | 机器人里程计信息 |
| `/swarm_trajs` | traj_utils/MultiBsplines | 其他机器人轨迹 |
| `/broadcast_bspline` | traj_utils/Bspline | 广播的B样条轨迹 |

### 发布话题  
| 话题名 | 消息类型 | 描述 |
|--------|----------|------|
| `/planning/bspline` | traj_utils/Bspline | 规划的B样条轨迹 |
| `/planning/data_display` | traj_utils/DataDisp | 调试数据显示 |
| `/broadcast_bspline` | traj_utils/Bspline | 向其他机器人广播轨迹 |

## 使用方法

### 1. 单机器人启动
```bash
# 启动规划节点
ros2 run ego_planner ego_planner_node \
  --ros-args \
  -p target_type:=1 \
  -p planning_horizen:=7.5
```

### 2. 多机器人启动
```bash
# 机器人0
ros2 run ego_planner ego_planner_node \
  --ros-args \
  -p drone_id:=0 \
  -p target_type:=2

# 机器人1  
ros2 run ego_planner ego_planner_node \
  --ros-args \
  -p drone_id:=1 \
  -p target_type:=2
```

### 3. RViz交互
- 启动RViz2
- 使用"2D Nav Goal"工具设置目标点
- 观察轨迹规划和执行过程

## 故障处理

### 常见问题
1. **规划失败**: 检查起终点可达性，调整规划参数
2. **频繁重规划**: 调整重规划阈值，检查感知精度
3. **多机器人冲突**: 检查通信连接，调整优先级机制
4. **轨迹不平滑**: 调整B样条优化参数

### 调试工具
- `/planning/data_display` 话题监控内部状态
- RViz可视化轨迹和路径点
- 状态机状态日志输出

## 性能特点

- **实时性**: 毫秒级轨迹重规划
- **鲁棒性**: 多层次故障检测和恢复
- **可扩展性**: 支持任意数量机器人协调
- **安全性**: 完备的紧急停止和碰撞避免机制 