# 机器人检测模块 (drone_detect)

## 概述

`drone_detect` 是EGO-Planner-Swarm系统中的多机器人检测模块，基于Fake Person Detection原理实现机器人的相互感知和检测。该模块通过处理来自激光雷达、相机等传感器的数据，识别和跟踪其他机器人的位置和运动状态，为多机器人协调提供感知基础。

## 主要功能

### 1. 机器人检测
- **点云目标检测**: 基于点云数据的机器人识别
- **视觉检测**: 使用相机图像进行机器人检测
- **特征匹配**: 通过特征点匹配识别特定机器人
- **多传感器融合**: 融合多种传感器的检测结果

### 2. 目标跟踪
- **卡尔曼滤波**: 平滑的目标状态估计
- **数据关联**: 多帧间的目标关联
- **运动预测**: 预测其他机器人的运动轨迹
- **遮挡处理**: 处理视觉遮挡情况

### 3. 状态估计
- **位置估计**: 3D空间中的精确位置
- **速度估计**: 目标的运动速度和方向
- **姿态估计**: 机器人的朝向信息
- **不确定性量化**: 估计结果的置信度

### 4. 通信接口
- **检测结果发布**: 向规划模块提供检测信息
- **可视化输出**: RViz中的检测结果显示
- **调试信息**: 丰富的调试和分析数据

## 核心组件

### DroneDetector类

```cpp
class DroneDetector {
public:
    // 初始化和配置
    void init(ros::NodeHandle& nh);
    void setParam(const DetectionParam& param);
    
    // 检测接口
    bool detectDrones(const sensor_msgs::PointCloud2& cloud,
                     vector<DetectedDrone>& detections);
    bool detectDrones(const sensor_msgs::Image& image,
                     vector<DetectedDrone>& detections);
    
    // 跟踪更新
    void updateTracking(const vector<DetectedDrone>& detections);
    vector<TrackedDrone> getTrackedDrones();
    
    // 状态查询
    bool getDroneState(int drone_id, DroneState& state);
    vector<DroneState> getAllDroneStates();
    
    // 预测功能
    DroneState predictDroneState(int drone_id, double time_ahead);
};
```

### DroneTracker类

```cpp
class DroneTracker {
public:
    // 跟踪器管理
    void initTracker(int drone_id, const DroneState& initial_state);
    void updateTracker(int drone_id, const DetectedDrone& detection);
    void removeTracker(int drone_id);
    
    // 状态估计
    DroneState getEstimatedState(int drone_id);
    Eigen::Matrix3d getCovariance(int drone_id);
    
    // 预测
    DroneState predict(int drone_id, double dt);
    void predictToTime(int drone_id, double target_time);
    
    // 数据关联
    void associateDetections(const vector<DetectedDrone>& detections);
    double computeAssociationCost(const TrackedDrone& track, 
                                 const DetectedDrone& detection);
};
```

### FakePersonDetector类

```cpp
class FakePersonDetector {
public:
    // 基于"Fake Person"的检测方法
    void detectFakePerson(const sensor_msgs::PointCloud2& cloud);
    void processDepthImage(const sensor_msgs::Image& depth_img);
    
    // 特征提取
    vector<Eigen::Vector3d> extractKeyPoints(const pcl::PointCloud<pcl::PointXYZ>& cloud);
    PersonFeature extractFeatures(const cv::Mat& image, const cv::Rect& bbox);
    
    // 分类器
    bool classifyAsDrone(const PersonFeature& feature);
    double computeConfidence(const PersonFeature& feature);
};
```

## 检测算法实现

### 1. 点云处理检测
```cpp
bool detectDronesFromPointCloud(const sensor_msgs::PointCloud2& cloud_msg) {
    // 1. 点云预处理
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(cloud_msg, cloud);
    
    // 2. 降采样和滤波
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setLeafSize(0.05, 0.05, 0.05);
    voxel_filter.setInputCloud(cloud.makeShared());
    voxel_filter.filter(cloud);
    
    // 3. 地面移除
    removeGroundPlane(cloud);
    
    // 4. 聚类分析
    vector<pcl::PointIndices> clusters;
    euclideanClustering(cloud, clusters);
    
    // 5. 机器人形状验证
    vector<DetectedDrone> detections;
    for (const auto& cluster : clusters) {
        if (validateDroneCluster(cloud, cluster)) {
            DetectedDrone drone = extractDroneFromCluster(cloud, cluster);
            detections.push_back(drone);
        }
    }
    
    // 6. 更新跟踪器
    updateTracking(detections);
    return !detections.empty();
}
```

### 2. 聚类和验证
```cpp
bool validateDroneCluster(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                         const pcl::PointIndices& cluster) {
    // 1. 尺寸检查
    if (cluster.indices.size() < min_cluster_size_ || 
        cluster.indices.size() > max_cluster_size_) {
        return false;
    }
    
    // 2. 计算包围盒
    Eigen::Vector3d min_pt, max_pt;
    computeBoundingBox(cloud, cluster, min_pt, max_pt);
    
    Eigen::Vector3d size = max_pt - min_pt;
    
    // 3. 尺寸约束检查
    if (size.x() < min_drone_size_.x() || size.x() > max_drone_size_.x() ||
        size.y() < min_drone_size_.y() || size.y() > max_drone_size_.y() ||
        size.z() < min_drone_size_.z() || size.z() > max_drone_size_.z()) {
        return false;
    }
    
    // 4. 形状特征检查
    double aspect_ratio = size.x() / size.y();
    if (aspect_ratio < min_aspect_ratio_ || aspect_ratio > max_aspect_ratio_) {
        return false;
    }
    
    // 5. 高度检查 (机器人应该在合理高度)
    Eigen::Vector3d center = (min_pt + max_pt) / 2.0;
    if (center.z() < min_flight_height_ || center.z() > max_flight_height_) {
        return false;
    }
    
    return true;
}
```

### 3. 特征提取
```cpp
DetectedDrone extractDroneFromCluster(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                                     const pcl::PointIndices& cluster) {
    DetectedDrone drone;
    
    // 1. 计算质心
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (int idx : cluster.indices) {
        centroid += Eigen::Vector3d(cloud.points[idx].x, 
                                   cloud.points[idx].y, 
                                   cloud.points[idx].z);
    }
    centroid /= cluster.indices.size();
    drone.position = centroid;
    
    // 2. 主成分分析
    Eigen::Matrix3d covariance;
    computeCovariance(cloud, cluster, centroid, covariance);
    
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    Eigen::Vector3d eigenvalues = solver.eigenvalues();
    Eigen::Matrix3d eigenvectors = solver.eigenvectors();
    
    // 3. 朝向估计
    drone.orientation = extractOrientation(eigenvectors);
    
    // 4. 置信度计算
    drone.confidence = computeDetectionConfidence(eigenvalues, cluster.indices.size());
    
    // 5. 时间戳
    drone.timestamp = ros::Time::now();
    
    return drone;
}
```

## 卡尔曼滤波跟踪

### 1. 状态模型
```cpp
class DroneKalmanFilter {
private:
    // 状态向量 [x, y, z, vx, vy, vz, ax, ay, az]
    Eigen::VectorXd state_;        // 9x1 状态向量
    Eigen::MatrixXd P_;            // 9x9 协方差矩阵
    Eigen::MatrixXd F_;            // 9x9 状态转移矩阵
    Eigen::MatrixXd Q_;            // 9x9 过程噪声协方差
    Eigen::MatrixXd H_;            // 3x9 观测矩阵
    Eigen::MatrixXd R_;            // 3x3 观测噪声协方差
    
public:
    void predict(double dt) {
        // 1. 更新状态转移矩阵
        updateTransitionMatrix(dt);
        
        // 2. 预测步骤
        state_ = F_ * state_;
        P_ = F_ * P_ * F_.transpose() + Q_;
    }
    
    void update(const Eigen::Vector3d& measurement) {
        // 1. 创新计算
        Eigen::Vector3d innovation = measurement - H_ * state_;
        
        // 2. 创新协方差
        Eigen::Matrix3d S = H_ * P_ * H_.transpose() + R_;
        
        // 3. 卡尔曼增益
        Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
        
        // 4. 状态更新
        state_ = state_ + K * innovation;
        
        // 5. 协方差更新
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(9, 9);
        P_ = (I - K * H_) * P_;
    }
};
```

### 2. 数据关联
```cpp
void associateDetections(const vector<DetectedDrone>& detections) {
    // 1. 计算代价矩阵
    Eigen::MatrixXd cost_matrix(trackers_.size(), detections.size());
    
    for (size_t i = 0; i < trackers_.size(); ++i) {
        for (size_t j = 0; j < detections.size(); ++j) {
            cost_matrix(i, j) = computeAssociationCost(trackers_[i], detections[j]);
        }
    }
    
    // 2. 匈牙利算法求解最优关联
    vector<int> assignment = hungarianAlgorithm(cost_matrix);
    
    // 3. 更新关联的跟踪器
    for (size_t i = 0; i < assignment.size(); ++i) {
        if (assignment[i] >= 0 && cost_matrix(i, assignment[i]) < max_association_cost_) {
            trackers_[i].update(detections[assignment[i]]);
        } else {
            trackers_[i].predict_only();  // 仅预测，无观测更新
        }
    }
    
    // 4. 初始化新的跟踪器
    vector<bool> detection_used(detections.size(), false);
    for (int assign : assignment) {
        if (assign >= 0) detection_used[assign] = true;
    }
    
    for (size_t j = 0; j < detections.size(); ++j) {
        if (!detection_used[j]) {
            initNewTracker(detections[j]);
        }
    }
}
```

## 多传感器融合

### 1. 传感器数据融合
```cpp
void fuseSensorData(const sensor_msgs::PointCloud2& lidar_data,
                   const sensor_msgs::Image& camera_data) {
    // 1. 分别进行检测
    vector<DetectedDrone> lidar_detections, camera_detections;
    detectFromLidar(lidar_data, lidar_detections);
    detectFromCamera(camera_data, camera_detections);
    
    // 2. 空间关联
    vector<FusedDetection> fused_detections;
    associateCrossSensor(lidar_detections, camera_detections, fused_detections);
    
    // 3. 融合估计
    for (auto& fused : fused_detections) {
        if (fused.has_lidar && fused.has_camera) {
            // 加权融合
            fusedPosition = weightedFusion(fused.lidar_pos, fused.camera_pos,
                                         lidar_weight_, camera_weight_);
        } else if (fused.has_lidar) {
            fusedPosition = fused.lidar_pos;
        } else {
            fusedPosition = fused.camera_pos;
        }
    }
}
```

### 2. 置信度融合
```cpp
double fuseConfidence(double lidar_confidence, double camera_confidence) {
    // Dempster-Shafer证据理论融合
    double m1 = lidar_confidence;      // 激光雷达的基本概率分配
    double m2 = camera_confidence;     // 相机的基本概率分配
    
    double conflict = m1 * (1 - m2) + (1 - m1) * m2;  // 冲突度
    
    if (conflict < 1.0) {
        double fused = (m1 * m2) / (1 - conflict);
        return std::min(1.0, fused);
    } else {
        return std::max(m1, m2);  // 冲突过大时取最大值
    }
}
```

## 配置参数

### 检测参数
```yaml
# 点云处理
voxel_leaf_size: 0.05        # 体素滤波叶子尺寸(m)
cluster_tolerance: 0.3       # 聚类容忍度(m)
min_cluster_size: 50         # 最小聚类尺寸
max_cluster_size: 5000       # 最大聚类尺寸

# 机器人尺寸约束
min_drone_size: [0.2, 0.2, 0.1]  # 最小机器人尺寸(m)
max_drone_size: [2.0, 2.0, 1.0]  # 最大机器人尺寸(m)
min_aspect_ratio: 0.5        # 最小长宽比
max_aspect_ratio: 3.0        # 最大长宽比

# 飞行高度约束
min_flight_height: 0.3       # 最小飞行高度(m)
max_flight_height: 5.0       # 最大飞行高度(m)
```

### 跟踪参数
```yaml
# 卡尔曼滤波器
process_noise_std: 0.1       # 过程噪声标准差
measurement_noise_std: 0.05  # 观测噪声标准差
initial_position_std: 0.5    # 初始位置不确定性
initial_velocity_std: 1.0    # 初始速度不确定性

# 数据关联
max_association_cost: 2.0    # 最大关联代价
max_prediction_time: 1.0     # 最大预测时间(s)
track_init_threshold: 3      # 跟踪器初始化阈值
track_delete_threshold: 5    # 跟踪器删除阈值
```

### 融合参数
```yaml
# 传感器权重
lidar_weight: 0.7           # 激光雷达权重
camera_weight: 0.3          # 相机权重
fusion_distance_threshold: 1.0  # 融合距离阈值(m)

# 置信度
min_detection_confidence: 0.3    # 最小检测置信度
track_confirmation_threshold: 0.7 # 跟踪确认阈值
```

## ROS2接口

### 订阅话题
| 话题名 | 消息类型 | 描述 |
|--------|----------|------|
| `/cloud` | sensor_msgs/PointCloud2 | 激光雷达点云数据 |
| `/camera/image` | sensor_msgs/Image | 相机图像数据 |
| `/camera/depth` | sensor_msgs/Image | 深度图像数据 |

### 发布话题
| 话题名 | 消息类型 | 描述 |
|--------|----------|------|
| `/drone_detect/detections` | drone_detect/DetectionArray | 检测结果 |
| `/drone_detect/tracks` | drone_detect/TrackArray | 跟踪结果 |
| `/drone_detect/markers` | visualization_msgs/MarkerArray | 可视化标记 |

### 服务接口
| 服务名 | 服务类型 | 描述 |
|--------|----------|------|
| `/drone_detect/get_drone_state` | drone_detect/GetDroneState | 查询机器人状态 |
| `/drone_detect/predict_position` | drone_detect/PredictPosition | 位置预测服务 |

## 使用示例

### 1. 基本检测
```cpp
// 初始化检测器
DroneDetector detector;
detector.init(node);

// 检测回调函数
void cloudCallback(const sensor_msgs::PointCloud2& msg) {
    vector<DetectedDrone> detections;
    if (detector.detectDrones(msg, detections)) {
        for (const auto& drone : detections) {
            RCLCPP_INFO(node->get_logger(), 
                       "Detected drone at [%.2f, %.2f, %.2f] with confidence %.2f",
                       drone.position.x(), drone.position.y(), drone.position.z(),
                       drone.confidence);
        }
    }
}
```

### 2. 跟踪查询
```cpp
// 查询所有跟踪的机器人
vector<DroneState> tracked_drones = detector.getAllDroneStates();

for (const auto& drone : tracked_drones) {
    // 预测未来位置
    DroneState future_state = detector.predictDroneState(drone.id, 1.0);
    
    // 使用预测结果进行规划
    planAvoidance(future_state.position);
}
```

## 调试与可视化

### 1. RViz可视化
- 检测边界框显示
- 跟踪轨迹可视化
- 置信度热力图
- 预测轨迹显示

### 2. 性能监控
- 检测率统计
- 跟踪精度评估
- 计算时间分析
- 传感器融合效果

## 算法特点

- **鲁棒性**: 多传感器融合提高检测可靠性
- **实时性**: 高频率检测和跟踪更新 (>20Hz)
- **精确性**: 亚米级的位置估计精度
- **适应性**: 处理遮挡、动态环境等复杂情况
- **可扩展性**: 支持添加新的传感器模态
