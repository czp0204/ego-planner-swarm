# 规划环境模块 (plan_env)

## 概述

`plan_env` 是EGO-Planner-Swarm系统中的环境感知和地图管理模块，负责构建和维护机器人规划所需的3D环境表示。该模块融合多传感器数据，提供占用网格地图、距离场计算、动态障碍物预测等功能，为路径搜索和轨迹优化提供准确的环境信息。

## 主要功能

### 1. 3D占用网格地图
- **点云融合**: 激光雷达点云数据处理和融合
- **深度图融合**: RGB-D相机深度信息集成
- **概率占用**: 基于贝叶斯更新的占用概率估计
- **地图膨胀**: 考虑机器人尺寸的安全膨胀

### 2. 欧几里得距离场 (ESDF)
- **距离计算**: 快速距离场生成和更新
- **梯度计算**: 支持基于梯度的轨迹优化
- **增量更新**: 局部地图变化的高效更新
- **多分辨率**: 不同精度需求的距离场表示

### 3. 动态障碍物感知
- **物体检测**: 从传感器数据中分离动态物体
- **轨迹预测**: 基于历史数据的运动预测
- **多项式拟合**: 平滑轨迹建模
- **恒速模型**: 简化的线性运动预测

### 4. 环境建模工具
- **射线投射**: 高效的可见性计算
- **地图可视化**: RViz中的3D地图显示
- **性能监控**: 地图更新时间和内存使用统计

## 核心组件

### GridMap类

```cpp
class GridMap {
public:
    // 地图初始化和管理
    void initMap(rclcpp::Node::SharedPtr node);
    void resetBuffer();
    void resetBuffer(Eigen::Vector3d min, Eigen::Vector3d max);
    
    // 坐标转换
    void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
    void indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos);
    int toAddress(const Eigen::Vector3i& id);
    
    // 占用状态查询
    int getOccupancy(Eigen::Vector3d pos);
    int getInflateOccupancy(Eigen::Vector3d pos);
    bool isInMap(const Eigen::Vector3d& pos);
    bool isUnknown(const Eigen::Vector3d& pos);
    bool isKnownFree(const Eigen::Vector3d& pos);
    bool isKnownOccupied(const Eigen::Vector3d& pos);
    
    // 占用状态设置
    void setOccupancy(Eigen::Vector3d pos, double occ = 1);
    void setOccupied(Eigen::Vector3d pos);
    
    // 地图信息获取
    double getResolution();
    Eigen::Vector3d getOrigin();
    void getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size);
};
```

### ObjPredictor类

```cpp
class ObjPredictor : public rclcpp::Node {
public:
    // 初始化和配置
    void init();
    
    // 预测接口
    ObjPrediction getPredictionTraj();
    ObjScale getObjScale();
    
    // 轨迹评估
    Eigen::Vector3d evaluatePoly(int obs_id, double time);
    Eigen::Vector3d evaluateConstVel(int obs_id, double time);
    
    // 状态查询
    int getObjNums();
};
```

### RayCaster类

```cpp
class RayCaster {
public:
    // 射线投射核心算法
    bool setInput(const Eigen::Vector3d& start, const Eigen::Vector3d& end);
    bool step(Eigen::Vector3d& ray_pt);
    
    // 可见性检测
    vector<Eigen::Vector3d> raycast(const Eigen::Vector3d& start, 
                                   const Eigen::Vector3d& end);
    bool isVisible(const Eigen::Vector3d& start, 
                   const Eigen::Vector3d& end);
};
```

## 算法实现

### 1. 概率占用地图更新

```cpp
void updateOccupancyProbability(const Eigen::Vector3d& sensor_pos,
                               const vector<Eigen::Vector3d>& point_cloud) {
    for (const auto& point : point_cloud) {
        // 1. 射线投射到命中点
        vector<Eigen::Vector3d> ray_points = raycast(sensor_pos, point);
        
        // 2. 更新射线路径上的自由空间
        for (const auto& ray_pt : ray_points) {
            double old_logit = getOccupancyLogit(ray_pt);
            double new_logit = old_logit + prob_miss_log_;
            new_logit = std::max(new_logit, clamp_min_log_);
            setOccupancyLogit(ray_pt, new_logit);
        }
        
        // 3. 更新命中点的占用概率
        double hit_logit = getOccupancyLogit(point);
        hit_logit += prob_hit_log_;
        hit_logit = std::min(hit_logit, clamp_max_log_);
        setOccupancyLogit(point, hit_logit);
    }
}
```

### 2. ESDF距离场计算

```cpp
void computeESDF() {
    // 1. 初始化距离场
    distance_buffer_.assign(map_size_3d_, std::numeric_limits<double>::max());
    
    // 2. 种子点初始化（占用体素）
    queue<Eigen::Vector3i> seed_queue;
    for (int x = 0; x < map_voxel_num_.x(); ++x) {
        for (int y = 0; y < map_voxel_num_.y(); ++y) {
            for (int z = 0; z < map_voxel_num_.z(); ++z) {
                Eigen::Vector3i idx(x, y, z);
                if (isKnownOccupied(idx)) {
                    distance_buffer_[toAddress(idx)] = 0.0;
                    seed_queue.push(idx);
                }
            }
        }
    }
    
    // 3. 快速扫掠算法（Fast Sweeping）
    while (!seed_queue.empty()) {
        Eigen::Vector3i current = seed_queue.front();
        seed_queue.pop();
        
        // 遍历26邻域
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                for (int dz = -1; dz <= 1; dz++) {
                    if (dx == 0 && dy == 0 && dz == 0) continue;
                    
                    Eigen::Vector3i neighbor = current + Eigen::Vector3i(dx, dy, dz);
                    if (!isInMap(neighbor)) continue;
                    
                    double edge_cost = sqrt(dx*dx + dy*dy + dz*dz) * resolution_;
                    double new_dist = distance_buffer_[toAddress(current)] + edge_cost;
                    
                    if (new_dist < distance_buffer_[toAddress(neighbor)]) {
                        distance_buffer_[toAddress(neighbor)] = new_dist;
                        seed_queue.push(neighbor);
                    }
                }
            }
        }
    }
}
```

### 3. 动态物体轨迹预测

```cpp
void predictPolynomialTrajectory(const list<Eigen::Vector4d>& history) {
    if (history.size() < 4) return;  // 需要足够的历史数据
    
    // 1. 构建时间矩阵和观测向量
    int n = history.size();
    Eigen::MatrixXd A(n, 6);  // [1, t, t^2, t^3, t^4, t^5]
    Eigen::VectorXd bx(n), by(n), bz(n);
    
    int i = 0;
    for (const auto& point : history) {
        double t = point(3);  // 时间戳
        A.row(i) << 1, t, t*t, t*t*t, t*t*t*t, t*t*t*t*t;
        bx(i) = point(0);
        by(i) = point(1);
        bz(i) = point(2);
        i++;
    }
    
    // 2. 正则化最小二乘求解
    Eigen::MatrixXd AtA = A.transpose() * A;
    Eigen::MatrixXd regularizer = lambda_ * Eigen::MatrixXd::Identity(6, 6);
    Eigen::MatrixXd lhs = AtA + regularizer;
    
    // 3. 求解多项式系数
    Eigen::VectorXd coeffs_x = lhs.ldlt().solve(A.transpose() * bx);
    Eigen::VectorXd coeffs_y = lhs.ldlt().solve(A.transpose() * by);
    Eigen::VectorXd coeffs_z = lhs.ldlt().solve(A.transpose() * bz);
    
    // 4. 存储预测多项式
    vector<Eigen::Matrix<double, 6, 1>> polys(3);
    polys[0] = coeffs_x;
    polys[1] = coeffs_y;
    polys[2] = coeffs_z;
    
    prediction_.setPolynomial(polys);
}
```

### 4. 射线投射算法

```cpp
vector<Eigen::Vector3d> raycast(const Eigen::Vector3d& start, 
                               const Eigen::Vector3d& end) {
    vector<Eigen::Vector3d> ray_points;
    
    // 1. 3D DDA算法初始化
    Eigen::Vector3i start_idx, end_idx, current_idx;
    posToIndex(start, start_idx);
    posToIndex(end, end_idx);
    current_idx = start_idx;
    
    Eigen::Vector3i step;
    Eigen::Vector3d t_delta, t_max;
    
    // 2. 计算步进方向和增量
    for (int i = 0; i < 3; ++i) {
        if (end_idx[i] > start_idx[i]) {
            step[i] = 1;
            t_delta[i] = resolution_ / (end[i] - start[i]);
            t_max[i] = t_delta[i] * (start_idx[i] + 1.0 - start[i] / resolution_);
        } else if (end_idx[i] < start_idx[i]) {
            step[i] = -1;
            t_delta[i] = resolution_ / (start[i] - end[i]);
            t_max[i] = t_delta[i] * (start[i] / resolution_ - start_idx[i]);
        } else {
            step[i] = 0;
            t_delta[i] = std::numeric_limits<double>::max();
            t_max[i] = std::numeric_limits<double>::max();
        }
    }
    
    // 3. DDA主循环
    while (current_idx != end_idx) {
        // 添加当前体素
        Eigen::Vector3d current_pos;
        indexToPos(current_idx, current_pos);
        ray_points.push_back(current_pos);
        
        // 确定下一步方向
        int min_dim = 0;
        for (int i = 1; i < 3; ++i) {
            if (t_max[i] < t_max[min_dim]) {
                min_dim = i;
            }
        }
        
        // 步进到下一个体素
        current_idx[min_dim] += step[min_dim];
        t_max[min_dim] += t_delta[min_dim];
        
        // 边界检查
        if (!isInMap(current_idx)) break;
    }
    
    return ray_points;
}
```

## 多传感器融合

### 1. 深度图像处理
```cpp
void processDepthImage(const sensor_msgs::msg::Image& depth_msg) {
    // 1. 深度图像预处理
    cv::Mat depth_image = cv_bridge::toCvShare(depth_msg)->image;
    
    // 2. 深度滤波
    cv::Mat filtered_depth;
    cv::medianBlur(depth_image, filtered_depth, 5);
    
    // 3. 投影到3D点云
    vector<Eigen::Vector3d> projected_points;
    for (int v = 0; v < depth_image.rows; v += skip_pixel_) {
        for (int u = 0; u < depth_image.cols; u += skip_pixel_) {
            float depth = filtered_depth.at<float>(v, u);
            
            // 深度有效性检查
            if (depth < depth_filter_mindist_ || depth > depth_filter_maxdist_) {
                continue;
            }
            
            // 相机坐标系投影
            Eigen::Vector3d point_cam;
            point_cam.x() = (u - cx_) * depth / fx_;
            point_cam.y() = (v - cy_) * depth / fy_;
            point_cam.z() = depth;
            
            // 转换到世界坐标系
            Eigen::Vector3d point_world = camera_r_m_ * point_cam + camera_pos_;
            projected_points.push_back(point_world);
        }
    }
    
    // 4. 更新占用地图
    updateOccupancyFromPoints(camera_pos_, projected_points);
}
```

### 2. 点云数据融合
```cpp
void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstPtr& cloud_msg) {
    // 1. 点云预处理
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);
    
    // 2. 降采样
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setLeafSize(resolution_, resolution_, resolution_);
    voxel_filter.setInputCloud(cloud.makeShared());
    voxel_filter.filter(cloud);
    
    // 3. 转换为世界坐标
    vector<Eigen::Vector3d> world_points;
    for (const auto& point : cloud.points) {
        world_points.emplace_back(point.x, point.y, point.z);
    }
    
    // 4. 更新地图
    updateOccupancyFromPoints(current_odom_.position, world_points);
}
```

## 配置参数

### 地图参数
```yaml
# 地图尺寸和分辨率
map_size: [40.0, 40.0, 5.0]      # 地图尺寸 (m)
resolution: 0.1                   # 体素分辨率 (m)
local_update_range: [5.0, 5.0, 3.0]  # 局部更新范围 (m)
obstacles_inflation: 0.3          # 障碍物膨胀半径 (m)

# 坐标系
frame_id: "world"                 # 地图坐标系
pose_type: 2                      # 位姿类型 (1: PoseStamped, 2: Odometry)
```

### 传感器参数
```yaml
# 相机内参
fx: 387.229                       # 焦距x
fy: 387.229                       # 焦距y  
cx: 321.04                        # 主点x
cy: 243.44                        # 主点y

# 深度处理
depth_filter_mindist: 0.2         # 最小深度 (m)
depth_filter_maxdist: 5.0         # 最大深度 (m)
depth_filter_tolerance: 0.1       # 深度容忍度 (m)
skip_pixel: 2                     # 像素跳跃步长
```

### 占用概率参数
```yaml
# 概率更新
p_hit: 0.8                        # 命中概率
p_miss: 0.2                       # 未命中概率
p_min: 0.12                       # 最小占用概率
p_max: 0.97                       # 最大占用概率
p_occ: 0.8                        # 占用阈值概率

# 射线投射
min_ray_length: 0.1               # 最小射线长度 (m)
max_ray_length: 4.5               # 最大射线长度 (m)
```

### 动态物体预测参数
```yaml
# 预测参数
obj_num: 2                        # 动态物体数量
predict_rate: 20.0                # 预测频率 (Hz)
lambda: 1.0                       # 正则化参数
queue_size: 10                    # 历史队列大小
skip_num: 1                       # 跳跃帧数
```

## ROS2接口

### 订阅话题
| 话题名 | 消息类型 | 描述 |
|--------|----------|------|
| `/camera/depth/image_raw` | sensor_msgs/Image | 深度图像数据 |
| `/camera/pose` | geometry_msgs/PoseStamped | 相机位姿 |
| `/odom` | nav_msgs/Odometry | 机器人里程计 |
| `/cloud` | sensor_msgs/PointCloud2 | 点云数据 |
| `/obj_pose_{id}` | geometry_msgs/PoseStamped | 动态物体位姿 |

### 发布话题
| 话题名 | 消息类型 | 描述 |
|--------|----------|------|
| `/map_inflate` | sensor_msgs/PointCloud2 | 膨胀后的占用地图 |
| `/map_vis` | visualization_msgs/Marker | 地图可视化 |
| `/depth_cloud` | sensor_msgs/PointCloud2 | 深度投影点云 |
| `/predicted_trajs` | visualization_msgs/MarkerArray | 预测轨迹可视化 |

## 性能优化

### 1. 局部地图更新
```cpp
void updateLocalMap() {
    // 1. 确定更新区域
    Eigen::Vector3d robot_pos = getCurrentPosition();
    Eigen::Vector3d local_min = robot_pos - local_update_range_ / 2.0;
    Eigen::Vector3d local_max = robot_pos + local_update_range_ / 2.0;
    
    // 2. 边界裁剪
    local_min = local_min.cwiseMax(map_min_boundary_);
    local_max = local_max.cwiseMin(map_max_boundary_);
    
    // 3. 只更新局部区域
    Eigen::Vector3i min_idx, max_idx;
    posToIndex(local_min, min_idx);
    posToIndex(local_max, max_idx);
    
    for (int x = min_idx.x(); x <= max_idx.x(); ++x) {
        for (int y = min_idx.y(); y <= max_idx.y(); ++y) {
            for (int z = min_idx.z(); z <= max_idx.z(); ++z) {
                // 仅处理局部区域内的体素
                updateVoxel(Eigen::Vector3i(x, y, z));
            }
        }
    }
}
```

### 2. 并行处理
```cpp
void parallelMapUpdate() {
    const int num_threads = std::thread::hardware_concurrency();
    std::vector<std::thread> threads;
    
    // 按体素分块并行处理
    int total_voxels = map_voxel_num_.x() * map_voxel_num_.y() * map_voxel_num_.z();
    int voxels_per_thread = total_voxels / num_threads;
    
    for (int t = 0; t < num_threads; ++t) {
        threads.emplace_back([&, t]() {
            int start_idx = t * voxels_per_thread;
            int end_idx = (t == num_threads - 1) ? total_voxels : (t + 1) * voxels_per_thread;
            
            for (int i = start_idx; i < end_idx; ++i) {
                // 处理体素i
                processVoxel(i);
            }
        });
    }
    
    for (auto& thread : threads) {
        thread.join();
    }
}
```

## 使用示例

### 1. 基本地图构建
```cpp
// 初始化地图
auto grid_map = std::make_shared<GridMap>();
grid_map->initMap(node);

// 订阅传感器数据
auto cloud_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/cloud", 10, 
    [&](const sensor_msgs::msg::PointCloud2::ConstPtr msg) {
        grid_map->cloudCallback(msg);
    });

// 查询占用状态
Eigen::Vector3d query_point(1.0, 2.0, 0.5);
if (grid_map->isKnownOccupied(query_point)) {
    // 点被占用
} else if (grid_map->isKnownFree(query_point)) {
    // 点为自由空间
}
```

### 2. 动态物体预测
```cpp
// 初始化预测器
auto predictor = std::make_shared<ObjPredictor>();
predictor->init();

// 获取预测轨迹
auto predictions = predictor->getPredictionTraj();
for (size_t i = 0; i < predictions->size(); ++i) {
    double future_time = current_time + 2.0;  // 预测2秒后
    Eigen::Vector3d future_pos = predictor->evaluatePoly(i, future_time);
    // 使用预测位置进行避障规划
}
```

## 调试工具

### 1. 可视化
- 3D占用网格显示
- ESDF距离场颜色映射
- 动态物体轨迹显示
- 射线投射可视化

### 2. 性能监控
- 地图更新频率统计
- 内存使用监控
- 传感器数据延迟分析

## 算法特点

- **高效性**: 增量式地图更新，毫秒级响应
- **精确性**: 亚体素级的距离场精度
- **鲁棒性**: 多传感器融合提高可靠性
- **实时性**: 支持高频率的动态环境更新
- **可扩展性**: 模块化设计便于功能扩展 