# 路径搜索算法 (path_searching)

## 概述

`path_searching` 是EGO-Planner-Swarm系统中的路径搜索模块，实现了动态A*算法(Dynamic A*)用于多机器人环境下的路径规划。该模块专门处理动态环境中的快速路径搜索，为后续的轨迹优化提供初始可行路径。

## 主要功能

### 1. 动态A*搜索
- **启发式搜索**: 基于欧几里得距离的启发式函数
- **动态环境适应**: 考虑动态障碍物的路径搜索
- **多分辨率搜索**: 支持不同精度的网格搜索
- **实时重搜索**: 环境变化时的快速路径更新

### 2. 运动学约束
- **速度约束**: 考虑机器人最大速度限制
- **加速度约束**: 满足机器人动力学特性
- **转向约束**: 限制急转弯和不连续运动
- **时间最优**: 在约束条件下寻找时间最优路径

### 3. 多机器人协调
- **时空搜索**: 在时间-空间维度中进行路径搜索
- **优先级机制**: 基于机器人ID的搜索优先级
- **冲突避免**: 主动避开其他机器人的规划路径
- **死锁检测**: 检测并解决多机器人死锁情况

## 核心算法

### DynAstar类

```cpp
class DynAstar {
public:
    // 主要搜索接口
    int search(Eigen::Vector3d start_pt, Eigen::Vector3d start_v, 
               Eigen::Vector3d start_a, Eigen::Vector3d end_pt, 
               Eigen::Vector3d end_v, bool init, bool dynamic = false, 
               double time_start = -1);
    
    // 路径提取和处理
    void setParam(ros::NodeHandle& nh);
    vector<Eigen::Vector3d> getPath();
    vector<vector<Eigen::Vector3d>> getVisitedNodes();
    
    // 环境设置
    void setEnvironment(const std::shared_ptr<GridMap>& env);
    void init();
    void reset();
};
```

### 搜索节点结构

```cpp
struct PathNode {
    // 节点状态
    Eigen::Vector3i index;     // 网格索引
    Eigen::Vector3d position;  // 世界坐标位置
    double g_score, f_score;   // A*算法的g值和f值
    double time;               // 时间维度
    
    // 运动学状态
    Eigen::Vector3d velocity;
    Eigen::Vector3d acceleration;
    
    // 搜索相关
    PathNode* parent;          // 父节点指针
    int node_state;           // 节点状态(OPENSET/CLOSESET)
    
    // 比较函数 (用于优先队列)
    bool operator < (const PathNode& node) const {
        return f_score > node.f_score;
    }
};
```

## 算法流程

### 1. 搜索初始化
```cpp
int DynAstar::search(...) {
    // 1. 参数检查和初始化
    if (!checkInput(start_pt, start_v, start_a, end_pt, end_v)) {
        return NO_PATH;
    }
    
    // 2. 清空之前的搜索结果
    resetSearch();
    
    // 3. 设置起点和终点
    PathNode* start_node = path_node_pool_[0];
    PathNode* end_node = path_node_pool_[1];
    setStartAndEnd(start_node, end_node);
    
    // 4. 将起点加入开放集
    open_set_.push(start_node);
    return SEARCHING;
}
```

### 2. 主搜索循环
```cpp
while (!open_set_.empty()) {
    // 1. 取出f值最小的节点
    PathNode* current = open_set_.top();
    open_set_.pop();
    
    // 2. 检查是否到达目标
    if (isGoal(current)) {
        return REACH_END;
    }
    
    // 3. 扩展相邻节点
    expandNode(current);
}
```

### 3. 节点扩展
```cpp
void expandNode(PathNode* current) {
    // 1. 遍历所有可能的运动
    for (auto& motion : motion_primitives_) {
        // 2. 计算新状态
        Eigen::Vector3d new_pos = current->position + motion.delta_pos;
        Eigen::Vector3d new_vel = current->velocity + motion.delta_vel;
        
        // 3. 检查运动学约束
        if (!checkKinodynamicConstraints(new_pos, new_vel)) {
            continue;
        }
        
        // 4. 检查碰撞
        if (env_->getInflateOccupancy(new_pos) || 
            checkSwarmCollision(new_pos, current->time + motion.duration)) {
            continue;
        }
        
        // 5. 更新或创建邻居节点
        updateNeighbor(current, new_pos, new_vel, motion);
    }
}
```

## 运动学模型

### 1. 运动原语生成
```cpp
void generateMotionPrimitives() {
    motion_primitives_.clear();
    
    // 速度采样
    for (double vx = -max_vel_; vx <= max_vel_; vx += vel_resolution_) {
        for (double vy = -max_vel_; vy <= max_vel_; vy += vel_resolution_) {
            for (double vz = -max_vel_; vz <= max_vel_; vz += vel_resolution_) {
                MotionPrimitive primitive;
                primitive.delta_vel = Eigen::Vector3d(vx, vy, vz);
                primitive.duration = time_resolution_;
                primitive.delta_pos = primitive.delta_vel * primitive.duration;
                
                // 检查约束
                if (checkMotionPrimitive(primitive)) {
                    motion_primitives_.push_back(primitive);
                }
            }
        }
    }
}
```

### 2. 动力学约束检查
```cpp
bool checkKinodynamicConstraints(const Eigen::Vector3d& pos, 
                                const Eigen::Vector3d& vel,
                                const Eigen::Vector3d& acc) {
    // 1. 速度约束
    if (vel.norm() > max_vel_) {
        return false;
    }
    
    // 2. 加速度约束  
    if (acc.norm() > max_acc_) {
        return false;
    }
    
    // 3. 位置边界检查
    if (!env_->isInMap(pos)) {
        return false;
    }
    
    return true;
}
```

## 多机器人冲突避免

### 1. 时空冲突检测
```cpp
bool checkSwarmCollision(const Eigen::Vector3d& pos, double time) {
    for (auto& other_traj : swarm_trajs_) {
        // 1. 获取其他机器人在该时间的位置
        Eigen::Vector3d other_pos = other_traj.evaluatePos(time);
        
        // 2. 检查距离
        if ((pos - other_pos).norm() < safe_distance_) {
            return true; // 发生冲突
        }
    }
    return false;
}
```

### 2. 优先级搜索
```cpp
int prioritizedSearch() {
    // 1. 按优先级排序机器人
    std::sort(robots_.begin(), robots_.end(), 
              [](const Robot& a, const Robot& b) {
                  return a.priority > b.priority;
              });
    
    // 2. 依次为每个机器人规划路径
    for (auto& robot : robots_) {
        int result = robot.search();
        if (result != REACH_END) {
            return result; // 搜索失败
        }
        // 将已规划路径加入约束
        addTrajectoryConstraint(robot.getTrajectory());
    }
    
    return REACH_END;
}
```

## 配置参数

### 搜索参数
```yaml
# 网格分辨率
resolution: 0.1              # 空间分辨率(m)
time_resolution: 0.1         # 时间分辨率(s)

# 搜索范围
max_search_time: 0.1         # 最大搜索时间(s)
max_iterations: 10000        # 最大迭代次数

# 启发式权重
heuristic_weight: 1.0        # 启发式函数权重
time_weight: 10.0            # 时间权重
```

### 动力学参数
```yaml
# 速度约束
max_vel: 2.0                 # 最大速度(m/s)
max_acc: 2.0                 # 最大加速度(m/s²)
max_jerk: 4.0               # 最大跃度(m/s³)

# 安全参数  
safe_distance: 0.5          # 机器人间安全距离(m)
collision_check_resolution: 0.1  # 碰撞检测分辨率
```

### 优化参数
```yaml
# 搜索优化
use_jps: false              # 是否使用Jump Point Search
use_theta_star: false       # 是否使用Theta*优化
smooth_path: true           # 是否平滑路径

# 内存管理
max_node_num: 100000        # 最大节点数量
allocate_num: 100000        # 预分配节点数量
```

## 性能优化

### 1. 内存池管理
```cpp
class NodePool {
private:
    vector<PathNode*> pool_;
    int use_node_num_;
    int allocate_num_;
    
public:
    PathNode* getNode() {
        if (use_node_num_ >= allocate_num_) {
            return nullptr; // 内存不足
        }
        return pool_[use_node_num_++];
    }
    
    void reset() {
        use_node_num_ = 0;
    }
};
```

### 2. 启发式函数优化
```cpp
double calculateHeuristic(const Eigen::Vector3d& pos1, 
                         const Eigen::Vector3d& pos2) {
    // 1. 欧几里得距离
    double dist = (pos1 - pos2).norm();
    
    // 2. 考虑运动学约束的时间估计
    double time_est = dist / max_vel_;
    
    // 3. 加权组合
    return heuristic_weight_ * dist + time_weight_ * time_est;
}
```

## 依赖项

- **ROS2核心**: rclcpp, rclpy, std_msgs, visualization_msgs, nav_msgs
- **数学库**: Eigen3
- **点云处理**: PCL, pcl_conversions
- **环境表示**: plan_env (GridMap)
- **图像处理**: cv_bridge

## 使用示例

### 1. 基本搜索
```cpp
// 初始化搜索器
DynAstar searcher;
searcher.setEnvironment(env_ptr);
searcher.init();

// 执行搜索
Eigen::Vector3d start(0, 0, 1);
Eigen::Vector3d end(10, 10, 1);
int result = searcher.search(start, start_v, start_a, end, end_v, true);

if (result == DynAstar::REACH_END) {
    vector<Eigen::Vector3d> path = searcher.getPath();
    // 使用搜索结果
}
```

### 2. 多机器人搜索
```cpp
// 设置其他机器人轨迹
for (auto& traj : other_robot_trajs) {
    searcher.addSwarmTrajectory(traj);
}

// 执行优先级搜索
int result = searcher.prioritizedSearch();
```

## 调试工具

### 1. 可视化
- 搜索节点可视化 (`/path_searching/visited_nodes`)
- 路径结果可视化 (`/path_searching/path`)
- 运动原语可视化 (`/path_searching/primitives`)

### 2. 性能监控
- 搜索时间统计
- 扩展节点数量
- 内存使用情况

## 算法特点

- **实时性**: 毫秒级路径搜索
- **完备性**: 保证找到可行路径(如果存在)
- **最优性**: 在运动学约束下寻找最优路径  
- **鲁棒性**: 处理动态环境和多机器人冲突
- **可扩展性**: 支持不同的运动学模型和约束 