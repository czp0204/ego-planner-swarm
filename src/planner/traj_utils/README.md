# 轨迹工具库 (traj_utils)

## 概述

`traj_utils` 是EGO-Planner-Swarm系统中的轨迹工具和消息定义模块，提供了轨迹表示、处理、可视化等核心功能。该模块定义了B样条轨迹、多机器人轨迹集合等关键消息类型，以及轨迹操作的实用工具函数，为整个规划系统提供统一的轨迹数据结构和处理接口。

## 主要功能

### 1. 轨迹消息定义
- **Bspline消息**: 单机器人B样条轨迹表示
- **MultiBsplines消息**: 多机器人轨迹集合
- **DataDisp消息**: 调试和可视化数据传输
- **PolyTraj消息**: 多项式轨迹表示

### 2. 轨迹数据结构
- **控制点管理**: B样条控制点的存储和操作
- **时间参数化**: 轨迹时间信息的管理
- **多维轨迹**: 支持位置、速度、加速度轨迹
- **轨迹段连接**: 多段轨迹的拼接和管理

### 3. 轨迹处理工具
- **轨迹评估**: 在指定时间点计算轨迹状态
- **轨迹微分**: 计算轨迹的导数信息
- **轨迹转换**: 不同轨迹表示间的转换
- **轨迹验证**: 轨迹有效性和安全性检查

### 4. 可视化支持
- **轨迹绘制**: RViz中的轨迹可视化
- **控制点显示**: B样条控制点的可视化
- **调试信息**: 规划过程的调试数据显示
- **多机器人标识**: 不同机器人轨迹的区分显示

## 核心消息定义

### 1. Bspline.msg
```
# B样条轨迹消息定义
Header header

# 机器人标识
int32 drone_id

# B样条参数
int32 order          # B样条阶数
float64 start_time   # 轨迹起始时间
float64[] knots      # 节点向量
geometry_msgs/Point[] pos_pts    # 位置控制点
geometry_msgs/Vector3[] vel_pts  # 速度控制点  
geometry_msgs/Vector3[] acc_pts  # 加速度控制点

# 轨迹属性
float64 yaw_dt       # 偏航角时间步长
float64[] yaw_pts    # 偏航角控制点
```

### 2. MultiBsplines.msg
```
# 多机器人轨迹集合消息
Header header

# 轨迹数组
Bspline[] traj       # 多个机器人的轨迹

# 全局信息  
int32 drone_id_from  # 发送方机器人ID
float64 start_time   # 全局起始时间
```

### 3. DataDisp.msg
```
# 调试和可视化数据消息
Header header

# 路径点信息
geometry_msgs/Point[] path          # 规划路径
geometry_msgs/Point[] start_end     # 起终点
geometry_msgs/Point[] kino_path     # 运动学路径

# 调试信息
geometry_msgs/Point[] pred_obj      # 预测障碍物
geometry_msgs/Point[] guide_path    # 引导路径
geometry_msgs/Point[] init_path     # 初始路径

# 可视化参数
int32 path_size                     # 路径点数量
int32 start_end_size               # 起终点数量
int32 kino_path_size               # 运动学路径点数量
```

## 轨迹数学表示

### 1. B样条基础理论

B样条曲线的数学表达式：
```
P(t) = Σ(i=0 to n) N_i,p(t) * P_i

其中：
- P(t): t时刻的轨迹点
- N_i,p(t): i阶p次B样条基函数
- P_i: 第i个控制点
- n: 控制点数量-1
- p: B样条阶数
```

### 2. de Boor算法实现

```cpp
// de Boor递推算法计算B样条值
Eigen::Vector3d evaluateDeBoor(double t, const vector<Eigen::Vector3d>& ctrl_pts,
                               const vector<double>& knots, int degree) {
    // 1. 找到对应的节点区间
    int span = findKnotSpan(t, knots, degree);
    
    // 2. 计算非零基函数值
    vector<double> basis_vals;
    computeBasisFunctions(span, t, knots, degree, basis_vals);
    
    // 3. 计算轨迹点
    Eigen::Vector3d point = Eigen::Vector3d::Zero();
    for (int i = 0; i <= degree; ++i) {
        point += basis_vals[i] * ctrl_pts[span - degree + i];
    }
    
    return point;
}
```

### 3. 轨迹导数计算

```cpp
// 计算B样条轨迹的r阶导数
vector<Eigen::Vector3d> computeDerivativeControlPoints(
    const vector<Eigen::Vector3d>& ctrl_pts,
    const vector<double>& knots, int degree, int r) {
    
    if (r == 0) return ctrl_pts;
    
    int n = ctrl_pts.size();
    vector<Eigen::Vector3d> deriv_pts(n - 1);
    
    for (int i = 0; i < n - 1; ++i) {
        double denominator = knots[i + degree + 1] - knots[i + 1];
        if (abs(denominator) > 1e-6) {
            deriv_pts[i] = degree * (ctrl_pts[i + 1] - ctrl_pts[i]) / denominator;
        } else {
            deriv_pts[i] = Eigen::Vector3d::Zero();
        }
    }
    
    // 递归计算高阶导数
    if (r > 1) {
        vector<double> new_knots(knots.begin() + 1, knots.end() - 1);
        return computeDerivativeControlPoints(deriv_pts, new_knots, degree - 1, r - 1);
    }
    
    return deriv_pts;
}
```

## 轨迹工具函数

### 1. 轨迹评估器

```cpp
class TrajectoryEvaluator {
public:
    // 构造函数
    TrajectoryEvaluator(const traj_utils::msg::Bspline& bspline_msg) {
        extractTrajectoryData(bspline_msg);
    }
    
    // 位置评估
    Eigen::Vector3d evaluatePosition(double t) {
        if (t < start_time_ || t > end_time_) {
            return Eigen::Vector3d::Zero();
        }
        return evaluateDeBoor(t, pos_ctrl_pts_, knots_, order_);
    }
    
    // 速度评估
    Eigen::Vector3d evaluateVelocity(double t) {
        if (t < start_time_ || t > end_time_) {
            return Eigen::Vector3d::Zero();
        }
        return evaluateDeBoor(t, vel_ctrl_pts_, vel_knots_, order_ - 1);
    }
    
    // 加速度评估
    Eigen::Vector3d evaluateAcceleration(double t) {
        if (t < start_time_ || t > end_time_) {
            return Eigen::Vector3d::Zero();
        }
        return evaluateDeBoor(t, acc_ctrl_pts_, acc_knots_, order_ - 2);
    }
    
    // 偏航角评估
    double evaluateYaw(double t) {
        if (yaw_ctrl_pts_.empty()) return 0.0;
        
        int idx = static_cast<int>((t - start_time_) / yaw_dt_);
        idx = std::max(0, std::min(idx, static_cast<int>(yaw_ctrl_pts_.size()) - 2));
        
        double ratio = (t - start_time_) / yaw_dt_ - idx;
        return yaw_ctrl_pts_[idx] * (1 - ratio) + yaw_ctrl_pts_[idx + 1] * ratio;
    }
    
private:
    vector<Eigen::Vector3d> pos_ctrl_pts_, vel_ctrl_pts_, acc_ctrl_pts_;
    vector<double> knots_, vel_knots_, acc_knots_;
    vector<double> yaw_ctrl_pts_;
    double start_time_, end_time_, yaw_dt_;
    int order_;
};
```

### 2. 轨迹转换器

```cpp
class TrajectoryConverter {
public:
    // B样条转多项式轨迹
    static vector<PolynomialTrajectory> bsplineToPolynomial(
        const traj_utils::msg::Bspline& bspline) {
        
        vector<PolynomialTrajectory> poly_segs;
        TrajectoryEvaluator evaluator(bspline);
        
        // 将B样条分段转换为多项式
        double dt = 0.01;
        for (double t = bspline.start_time; t < getEndTime(bspline); t += dt) {
            // 采样B样条轨迹
            auto pos = evaluator.evaluatePosition(t);
            auto vel = evaluator.evaluateVelocity(t);
            auto acc = evaluator.evaluateAcceleration(t);
            
            // 拟合多项式段
            PolynomialTrajectory poly_seg = fitPolynomial(pos, vel, acc, dt);
            poly_segs.push_back(poly_seg);
        }
        
        return poly_segs;
    }
    
    // 路径点转B样条
    static traj_utils::msg::Bspline pathToBspline(
        const vector<Eigen::Vector3d>& path_points,
        double time_duration, int order = 3) {
        
        traj_utils::msg::Bspline bspline_msg;
        
        // 1. 生成时间参数化
        vector<double> time_params = generateTimeParameters(path_points, time_duration);
        
        // 2. 生成节点向量
        vector<double> knots = generateKnotVector(path_points.size(), order);
        
        // 3. 控制点拟合
        vector<Eigen::Vector3d> ctrl_pts = fitControlPoints(path_points, time_params, knots, order);
        
        // 4. 填充消息
        fillBsplineMessage(bspline_msg, ctrl_pts, knots, order, time_duration);
        
        return bspline_msg;
    }
};
```

### 3. 轨迹验证器

```cpp
class TrajectoryValidator {
public:
    // 动力学约束检查
    static bool checkDynamicConstraints(const traj_utils::msg::Bspline& bspline,
                                       double max_vel, double max_acc) {
        TrajectoryEvaluator evaluator(bspline);
        double dt = 0.01;
        
        for (double t = bspline.start_time; t <= getEndTime(bspline); t += dt) {
            // 检查速度约束
            Eigen::Vector3d vel = evaluator.evaluateVelocity(t);
            if (vel.norm() > max_vel) {
                return false;
            }
            
            // 检查加速度约束
            Eigen::Vector3d acc = evaluator.evaluateAcceleration(t);
            if (acc.norm() > max_acc) {
                return false;
            }
        }
        
        return true;
    }
    
    // 碰撞检查
    static bool checkCollisionFree(const traj_utils::msg::Bspline& bspline,
                                  std::shared_ptr<GridMap> grid_map) {
        TrajectoryEvaluator evaluator(bspline);
        double dt = 0.02;
        
        for (double t = bspline.start_time; t <= getEndTime(bspline); t += dt) {
            Eigen::Vector3d pos = evaluator.evaluatePosition(t);
            
            if (grid_map->getInflateOccupancy(pos) == 1) {
                return false;  // 发生碰撞
            }
        }
        
        return true;
    }
    
    // 连续性检查
    static bool checkContinuity(const vector<traj_utils::msg::Bspline>& trajectory_segments) {
        const double eps = 1e-3;
        
        for (size_t i = 0; i < trajectory_segments.size() - 1; ++i) {
            TrajectoryEvaluator eval1(trajectory_segments[i]);
            TrajectoryEvaluator eval2(trajectory_segments[i + 1]);
            
            double t1_end = getEndTime(trajectory_segments[i]);
            double t2_start = trajectory_segments[i + 1].start_time;
            
            // 位置连续性
            Eigen::Vector3d pos1 = eval1.evaluatePosition(t1_end);
            Eigen::Vector3d pos2 = eval2.evaluatePosition(t2_start);
            if ((pos1 - pos2).norm() > eps) return false;
            
            // 速度连续性
            Eigen::Vector3d vel1 = eval1.evaluateVelocity(t1_end);
            Eigen::Vector3d vel2 = eval2.evaluateVelocity(t2_start);
            if ((vel1 - vel2).norm() > eps) return false;
        }
        
        return true;
    }
};
```

## 可视化工具

### 1. 轨迹可视化器

```cpp
class TrajectoryVisualizer {
private:
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    
public:
    TrajectoryVisualizer(rclcpp::Node::SharedPtr node) {
        marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/trajectory_markers", 10);
    }
    
    // 可视化单条轨迹
    void visualizeTrajectory(const traj_utils::msg::Bspline& bspline,
                           const std::string& ns = "trajectory",
                           const std_msgs::msg::ColorRGBA& color = createColor(1,0,0,1)) {
        
        visualization_msgs::msg::MarkerArray marker_array;
        
        // 1. 轨迹线
        auto traj_marker = createTrajectoryLineMarker(bspline, ns + "_line", color);
        marker_array.markers.push_back(traj_marker);
        
        // 2. 控制点
        auto ctrl_marker = createControlPointsMarker(bspline, ns + "_ctrl", color);
        marker_array.markers.push_back(ctrl_marker);
        
        // 3. 方向箭头
        auto arrow_markers = createDirectionArrows(bspline, ns + "_arrows", color);
        marker_array.markers.insert(marker_array.markers.end(), 
                                   arrow_markers.begin(), arrow_markers.end());
        
        marker_pub_->publish(marker_array);
    }
    
    // 可视化多机器人轨迹
    void visualizeSwarmTrajectories(const traj_utils::msg::MultiBsplines& multi_bsplines) {
        visualization_msgs::msg::MarkerArray marker_array;
        
        // 为每个机器人分配不同颜色
        vector<std_msgs::msg::ColorRGBA> colors = generateDistinctColors(multi_bsplines.traj.size());
        
        for (size_t i = 0; i < multi_bsplines.traj.size(); ++i) {
            const auto& bspline = multi_bsplines.traj[i];
            std::string ns = "drone_" + std::to_string(bspline.drone_id);
            
            // 轨迹线
            auto traj_marker = createTrajectoryLineMarker(bspline, ns, colors[i]);
            marker_array.markers.push_back(traj_marker);
            
            // 机器人位置标记
            auto robot_marker = createRobotMarker(bspline, ns + "_robot", colors[i]);
            marker_array.markers.push_back(robot_marker);
        }
        
        marker_pub_->publish(marker_array);
    }
    
private:
    // 创建轨迹线标记
    visualization_msgs::msg::Marker createTrajectoryLineMarker(
        const traj_utils::msg::Bspline& bspline,
        const std::string& ns,
        const std_msgs::msg::ColorRGBA& color) {
        
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = rclcpp::Clock().now();
        marker.ns = ns;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.05;  // 线宽
        marker.color = color;
        
        // 采样轨迹点
        TrajectoryEvaluator evaluator(bspline);
        double dt = 0.02;
        for (double t = bspline.start_time; t <= getEndTime(bspline); t += dt) {
            geometry_msgs::msg::Point point;
            Eigen::Vector3d pos = evaluator.evaluatePosition(t);
            point.x = pos.x();
            point.y = pos.y();
            point.z = pos.z();
            marker.points.push_back(point);
        }
        
        return marker;
    }
};
```

### 2. 调试数据可视化

```cpp
class DebugDataVisualizer {
public:
    // 可视化规划调试信息
    void visualizeDebugData(const traj_utils::msg::DataDisp& debug_data) {
        visualization_msgs::msg::MarkerArray marker_array;
        
        // 1. 规划路径
        if (!debug_data.path.empty()) {
            auto path_marker = createPathMarker(debug_data.path, "planning_path", 
                                              createColor(0, 1, 0, 0.8));
            marker_array.markers.push_back(path_marker);
        }
        
        // 2. 运动学路径
        if (!debug_data.kino_path.empty()) {
            auto kino_marker = createPathMarker(debug_data.kino_path, "kinodynamic_path",
                                              createColor(1, 1, 0, 0.8));
            marker_array.markers.push_back(kino_marker);
        }
        
        // 3. 起终点
        if (debug_data.start_end_size >= 2) {
            auto start_marker = createSphereMarker(debug_data.start_end[0], "start_point",
                                                 createColor(0, 1, 0, 1.0));
            auto end_marker = createSphereMarker(debug_data.start_end[1], "end_point",
                                               createColor(1, 0, 0, 1.0));
            marker_array.markers.push_back(start_marker);
            marker_array.markers.push_back(end_marker);
        }
        
        // 4. 预测障碍物
        if (!debug_data.pred_obj.empty()) {
            auto obj_marker = createCubeListMarker(debug_data.pred_obj, "predicted_obstacles",
                                                 createColor(1, 0, 1, 0.6));
            marker_array.markers.push_back(obj_marker);
        }
        
        debug_marker_pub_->publish(marker_array);
    }
};
```

## 配置和使用

### 1. CMakeLists.txt 配置
```cmake
# 依赖包
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# 消息生成
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Bspline.msg"
  "msg/MultiBsplines.msg"
  "msg/DataDisp.msg"
  "msg/PolyTraj.msg"
  DEPENDENCIES geometry_msgs std_msgs
)
```

### 2. 使用示例

```cpp
// 创建B样条轨迹
traj_utils::msg::Bspline bspline_msg;
bspline_msg.order = 3;
bspline_msg.start_time = current_time;
bspline_msg.drone_id = robot_id;

// 设置控制点
for (const auto& point : control_points) {
    geometry_msgs::msg::Point pt;
    pt.x = point.x(); pt.y = point.y(); pt.z = point.z();
    bspline_msg.pos_pts.push_back(pt);
}

// 发布轨迹
trajectory_pub_->publish(bspline_msg);

// 评估轨迹
TrajectoryEvaluator evaluator(bspline_msg);
double query_time = current_time + 1.0;
Eigen::Vector3d future_pos = evaluator.evaluatePosition(query_time);
Eigen::Vector3d future_vel = evaluator.evaluateVelocity(query_time);
```

## 依赖项

- **ROS2核心**: rclcpp, std_msgs, geometry_msgs, visualization_msgs
- **数学库**: Eigen3
- **消息系统**: rosidl_default_generators

## 算法特点

- **高效性**: 优化的B样条评估算法
- **精确性**: 数值稳定的de Boor算法实现
- **灵活性**: 支持多种轨迹表示和转换
- **实用性**: 丰富的工具函数和可视化支持
- **标准化**: 统一的消息格式便于系统集成