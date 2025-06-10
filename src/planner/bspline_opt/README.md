# B样条轨迹优化 (bspline_opt)

## 概述

`bspline_opt` 是EGO-Planner-Swarm系统中的轨迹优化核心模块，实现了基于B样条的轨迹优化算法。该模块将路径搜索得到的初始路径转换为平滑的B样条轨迹，并通过梯度优化方法满足动力学约束、安全约束和多机器人协调约束，生成高质量的可执行轨迹。

## 主要功能

### 1. B样条轨迹表示
- **均匀B样条**: 使用均匀节点向量的B样条曲线
- **分段连续性**: 保证C²连续性的平滑轨迹
- **局部控制**: 控制点的局部影响特性
- **实时修改**: 支持轨迹的实时调整和优化

### 2. 多目标优化
- **平滑性目标**: 最小化轨迹的加速度和急跃度
- **安全性约束**: 避免与静态和动态障碍物碰撞
- **动力学约束**: 满足速度、加速度等动力学限制
- **多机器人协调**: 避免与其他机器人轨迹冲突

### 3. 梯度优化算法
- **L-BFGS优化**: 拟牛顿法的高效实现
- **梯度下降**: 传统梯度下降算法
- **自适应步长**: 动态调整优化步长
- **早停机制**: 防止过度优化的停止条件

### 4. 约束处理
- **软约束**: 通过惩罚函数处理
- **硬约束**: 通过投影方法强制满足
- **优先级约束**: 不同约束的重要性排序
- **自适应权重**: 动态调整约束权重

## 核心组件

### BsplineOptimizer类

```cpp
class BsplineOptimizer {
public:
    // 优化接口
    bool BsplineOptimizeTraj(const vector<Eigen::Vector3d>& points, 
                            const double& ts, const int& cost_function, 
                            int max_num_id, int max_time_id);
    
    // B样条操作
    void setControlPoints(const Eigen::MatrixXd& points);
    Eigen::MatrixXd getControlPoints();
    
    // 约束设置
    void setEnvironment(const shared_ptr<GridMap>& env);
    void setSwarmTrajs(SwarmTrajData* swarm_trajs_ptr);
    void setTerminateCond(const int& max_num_id, const int& max_time_id);
    
    // 参数配置
    void setOptParam(ros::NodeHandle& nh);
    void setCostFunction(const int& cost_function);
    
    // 轨迹评估
    double getCostFunction();
    vector<Eigen::Vector3d> matrixToVectors(const Eigen::MatrixXd& ctrl_pts);
};
```

### UniformBspline类

```cpp
class UniformBspline {
public:
    // B样条构造
    UniformBspline(const Eigen::MatrixXd& points, const int& order, 
                   const double& interval);
    
    // 轨迹评估
    Eigen::Vector3d evaluateDeBoor(const double& t);
    Eigen::Vector3d evaluateDeBoorT(const double& t);
    
    // 导数计算
    UniformBspline getDerivative();
    double getTimeSum();
    double getInterval();
    
    // 轨迹操作
    void setUniformBspline(const Eigen::MatrixXd& points, 
                          const int& order, const double& interval);
    void getTimeSpan(double& um, double& um_p);
    
    // 重参数化
    UniformBspline reparameterize(double ratio);
    void adjustTime(double ratio);
};
```

### GradientDescentOptimizer类

```cpp
class GradientDescentOptimizer {
public:
    // 梯度下降优化
    bool optimize(Eigen::MatrixXd& control_points, 
                  double& final_cost, 
                  const int& max_iterations);
    
    // 梯度计算
    void computeGradient(const Eigen::MatrixXd& control_points,
                        Eigen::MatrixXd& gradient);
    
    // 步长调整
    double computeStepSize(const Eigen::MatrixXd& control_points,
                          const Eigen::MatrixXd& gradient);
    
    // 收敛检查
    bool checkConvergence(const Eigen::MatrixXd& gradient,
                         const double& cost_change);
};
```

## B样条数学原理

### 1. B样条基函数
```cpp
// de Boor-Cox递推公式
double BsplineBasis(int i, int p, double t, const vector<double>& knots) {
    if (p == 0) {
        return (knots[i] <= t && t < knots[i+1]) ? 1.0 : 0.0;
    }
    
    double alpha1 = (t - knots[i]) / (knots[i+p] - knots[i]);
    double alpha2 = (knots[i+p+1] - t) / (knots[i+p+1] - knots[i+1]);
    
    return alpha1 * BsplineBasis(i, p-1, t, knots) + 
           alpha2 * BsplineBasis(i+1, p-1, t, knots);
}
```

### 2. de Boor算法
```cpp
Eigen::Vector3d evaluateDeBoor(double t) {
    // 1. 找到合适的节点区间
    int span = findSpan(t);
    
    // 2. 计算非零基函数
    vector<double> basis(p_ + 1);
    computeBasisFunctions(span, t, basis);
    
    // 3. 线性组合计算结果
    Eigen::Vector3d result = Eigen::Vector3d::Zero();
    for (int j = 0; j <= p_; ++j) {
        result += basis[j] * control_points_.col(span - p_ + j);
    }
    
    return result;
}
```

## 代价函数设计

### 1. 平滑性代价
```cpp
double computeSmoothnesseCost(const UniformBspline& traj) {
    double cost = 0.0;
    double dt = 0.01;
    
    // 积分计算轨迹的二阶导数平方
    for (double t = 0; t <= traj.getTimeSum(); t += dt) {
        Eigen::Vector3d acc = traj.getDerivative().getDerivative().evaluateDeBoor(t);
        cost += acc.squaredNorm() * dt;
    }
    
    // 可选: 添加急跃度惩罚
    if (minimize_jerk_) {
        UniformBspline jerk_traj = traj.getDerivative().getDerivative().getDerivative();
        for (double t = 0; t <= traj.getTimeSum(); t += dt) {
            Eigen::Vector3d jerk = jerk_traj.evaluateDeBoor(t);
            cost += jerk_weight_ * jerk.squaredNorm() * dt;
        }
    }
    
    return cost;
}
```

### 2. 碰撞避免代价
```cpp
double computeCollisionCost(const UniformBspline& traj) {
    double cost = 0.0;
    double dt = 0.01;
    
    for (double t = 0; t <= traj.getTimeSum(); t += dt) {
        Eigen::Vector3d pos = traj.evaluateDeBoor(t);
        
        // 1. 静态障碍物碰撞
        double dist = env_->getDistance(pos);
        if (dist < safe_distance_) {
            double violation = safe_distance_ - dist;
            cost += collision_weight_ * violation * violation;
        }
        
        // 2. 动态障碍物碰撞
        cost += computeDynamicObstacleCost(pos, t);
    }
    
    return cost;
}

double computeDynamicObstacleCost(const Eigen::Vector3d& pos, double time) {
    double cost = 0.0;
    
    for (const auto& obs : dynamic_obstacles_) {
        Eigen::Vector3d obs_pos = obs.predictPosition(time);
        double dist = (pos - obs_pos).norm();
        
        if (dist < safe_distance_) {
            double violation = safe_distance_ - dist;
            cost += dynamic_obs_weight_ * violation * violation;
        }
    }
    
    return cost;
}
```

### 3. 动力学约束代价
```cpp
double computeDynamicsCost(const UniformBspline& traj) {
    double cost = 0.0;
    double dt = 0.01;
    
    UniformBspline vel_traj = traj.getDerivative();
    UniformBspline acc_traj = vel_traj.getDerivative();
    
    for (double t = 0; t <= traj.getTimeSum(); t += dt) {
        // 1. 速度约束
        Eigen::Vector3d vel = vel_traj.evaluateDeBoor(t);
        if (vel.norm() > max_vel_) {
            double violation = vel.norm() - max_vel_;
            cost += vel_weight_ * violation * violation;
        }
        
        // 2. 加速度约束
        Eigen::Vector3d acc = acc_traj.evaluateDeBoor(t);
        if (acc.norm() > max_acc_) {
            double violation = acc.norm() - max_acc_;
            cost += acc_weight_ * violation * violation;
        }
    }
    
    return cost;
}
```

### 4. 多机器人协调代价
```cpp
double computeSwarmCost(const UniformBspline& traj) {
    double cost = 0.0;
    double dt = 0.01;
    
    for (double t = 0; t <= traj.getTimeSum(); t += dt) {
        Eigen::Vector3d pos = traj.evaluateDeBoor(t);
        
        for (const auto& other_traj : swarm_trajs_) {
            if (other_traj.drone_id == my_id_) continue;
            
            // 获取其他机器人在时间t的位置
            Eigen::Vector3d other_pos = other_traj.evaluatePos(t);
            double dist = (pos - other_pos).norm();
            
            if (dist < swarm_safe_distance_) {
                double violation = swarm_safe_distance_ - dist;
                cost += swarm_weight_ * violation * violation;
            }
        }
    }
    
    return cost;
}
```

## 优化算法实现

### 1. L-BFGS优化
```cpp
bool optimizeWithLBFGS(Eigen::MatrixXd& control_points) {
    // 1. 初始化L-BFGS参数
    lbfgs_parameter_t params;
    lbfgs_parameter_init(&params);
    params.max_iterations = max_iterations_;
    params.epsilon = 1e-6;
    
    // 2. 转换为一维向量
    int n = control_points.size();
    lbfgsfloatval_t* x = lbfgs_malloc(n);
    Eigen::Map<Eigen::VectorXd>(x, n) = 
        Eigen::Map<Eigen::VectorXd>(control_points.data(), n);
    
    // 3. 执行优化
    lbfgsfloatval_t fx;
    int ret = lbfgs(n, x, &fx, evaluate, progress, this, &params);
    
    // 4. 转换回矩阵形式
    Eigen::Map<Eigen::VectorXd>(control_points.data(), n) = 
        Eigen::Map<Eigen::VectorXd>(x, n);
    
    lbfgs_free(x);
    return (ret == LBFGS_SUCCESS || ret == LBFGS_STOP || ret == LBFGS_ALREADY_MINIMIZED);
}
```

### 2. 梯度计算
```cpp
void computeTotalGradient(const Eigen::MatrixXd& control_points,
                         Eigen::MatrixXd& gradient) {
    gradient.setZero();
    
    // 1. 构建B样条轨迹
    UniformBspline traj(control_points, bspline_degree_, knot_span_);
    
    // 2. 计算各项梯度
    Eigen::MatrixXd smoothness_grad, collision_grad, dynamics_grad, swarm_grad;
    
    computeSmoothnessGradient(traj, smoothness_grad);
    computeCollisionGradient(traj, collision_grad);
    computeDynamicsGradient(traj, dynamics_grad);
    computeSwarmGradient(traj, swarm_grad);
    
    // 3. 加权组合
    gradient = lambda_smooth_ * smoothness_grad +
               lambda_collision_ * collision_grad +
               lambda_dynamics_ * dynamics_grad +
               lambda_swarm_ * swarm_grad;
    
    // 4. 固定起点和终点
    if (fix_start_) {
        gradient.col(0).setZero();
        gradient.col(1).setZero();
    }
    if (fix_end_) {
        gradient.col(gradient.cols()-1).setZero();
        gradient.col(gradient.cols()-2).setZero();
    }
}
```

### 3. 步长控制
```cpp
double computeOptimalStepSize(const Eigen::MatrixXd& control_points,
                             const Eigen::MatrixXd& gradient) {
    double alpha = initial_step_size_;
    double current_cost = computeTotalCost(control_points);
    
    // Armijo回溯线搜索
    const double c1 = 1e-4;  // Armijo常数
    const double rho = 0.5;  // 步长缩减因子
    
    for (int i = 0; i < max_line_search_iterations_; ++i) {
        Eigen::MatrixXd new_points = control_points - alpha * gradient;
        double new_cost = computeTotalCost(new_points);
        
        // Armijo条件检查
        double expected_decrease = c1 * alpha * gradient.squaredNorm();
        if (new_cost <= current_cost - expected_decrease) {
            return alpha;  // 满足Armijo条件
        }
        
        alpha *= rho;  // 缩减步长
    }
    
    return alpha;
}
```

## 配置参数

### 优化参数
```yaml
# 优化算法
optimization_method: "LBFGS"  # LBFGS, GradientDescent
max_iterations: 200           # 最大迭代次数
convergence_tolerance: 1e-6   # 收敛容忍度
initial_step_size: 1.0        # 初始步长

# B样条参数
bspline_degree: 3             # B样条阶数
knot_span: 0.1               # 节点间隔(s)
ctrl_pt_dist: 0.5            # 控制点间距(m)
```

### 代价函数权重
```yaml
# 平滑性
lambda_smooth: 1.0            # 平滑性权重
minimize_jerk: true           # 是否最小化急跃度

# 碰撞避免  
lambda_collision: 10.0        # 碰撞避免权重
safe_distance: 0.3            # 安全距离(m)

# 动力学约束
lambda_dynamics: 1.0          # 动力学约束权重
max_vel: 2.0                 # 最大速度(m/s)
max_acc: 2.0                 # 最大加速度(m/s²)

# 多机器人协调
lambda_swarm: 5.0            # 集群协调权重
swarm_safe_distance: 0.5     # 机器人间安全距离(m)
```

### 约束参数
```yaml
# 边界约束
fix_start: true              # 固定起点
fix_end: true                # 固定终点
start_vel: [0.0, 0.0, 0.0]   # 起点速度
end_vel: [0.0, 0.0, 0.0]     # 终点速度

# 时间约束
min_time_duration: 1.0       # 最小轨迹时间(s)
max_time_duration: 10.0      # 最大轨迹时间(s)
```

## 性能优化

### 1. 稀疏梯度计算
```cpp
void computeSparseGradient(const UniformBspline& traj,
                          Eigen::MatrixXd& gradient) {
    // 只计算受影响控制点的梯度
    for (int i = 0; i < control_points_.cols(); ++i) {
        // 确定控制点i影响的时间范围
        double t_start = std::max(0.0, (i - bspline_degree_) * knot_span_);
        double t_end = std::min(traj.getTimeSum(), (i + 1) * knot_span_);
        
        // 只在影响范围内计算梯度
        computeGradientInRange(traj, i, t_start, t_end, gradient);
    }
}
```

### 2. 并行梯度计算
```cpp
void computeParallelGradient(const UniformBspline& traj,
                            Eigen::MatrixXd& gradient) {
    const int num_threads = std::thread::hardware_concurrency();
    std::vector<std::thread> threads;
    std::vector<Eigen::MatrixXd> partial_gradients(num_threads);
    
    for (int t = 0; t < num_threads; ++t) {
        threads.emplace_back([&, t]() {
            int start_idx = t * control_points_.cols() / num_threads;
            int end_idx = (t + 1) * control_points_.cols() / num_threads;
            computeGradientPartial(traj, start_idx, end_idx, partial_gradients[t]);
        });
    }
    
    for (auto& thread : threads) {
        thread.join();
    }
    
    // 合并梯度
    gradient.setZero();
    for (const auto& grad : partial_gradients) {
        gradient += grad;
    }
}
```

## 使用示例

### 1. 基本优化
```cpp
// 初始化优化器
BsplineOptimizer optimizer;
optimizer.setEnvironment(env_ptr);
optimizer.setOptParam(node);

// 设置初始路径
vector<Eigen::Vector3d> path_points;
// ... 填充路径点

// 执行优化
bool success = optimizer.BsplineOptimizeTraj(
    path_points, 0.1, COST_FUNCTION::SMOOTHNESS_COLLISION, 50, 100);

if (success) {
    Eigen::MatrixXd optimized_points = optimizer.getControlPoints();
    // 使用优化结果
}
```

### 2. 多机器人优化
```cpp
// 设置其他机器人轨迹
SwarmTrajData swarm_data;
// ... 填充群体轨迹数据
optimizer.setSwarmTrajs(&swarm_data);

// 执行协调优化
bool success = optimizer.BsplineOptimizeTraj(
    path_points, 0.1, COST_FUNCTION::SMOOTHNESS_COLLISION_SWARM, 100, 200);
```

## 调试工具

### 1. 可视化
- 优化前后轨迹对比
- 控制点位置显示
- 代价函数值变化曲线
- 梯度方向可视化

### 2. 性能分析
- 优化收敛曲线
- 各代价项权重分析
- 计算时间统计

## 算法特点

- **高效性**: 毫秒级轨迹优化
- **平滑性**: 保证C²连续的平滑轨迹
- **安全性**: 完备的碰撞避免和动力学约束
- **协调性**: 多机器人冲突避免
- **实时性**: 支持在线轨迹调整和重优化 