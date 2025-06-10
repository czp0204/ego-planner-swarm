/**
 * @file grid_map.h
 * @brief EGO-Planner中的3D栅格占据地图实现
 * @author EGO-Planner团队
 * @date 2024
 * 
 * 该文件实现了基于体素的3D栅格占据地图，支持：
 * - 深度图像与里程计的融合建图
 * - 概率占据更新和光线投射
 * - 地图膨胀和碰撞检测
 * - 实时地图可视化和发布
 */
#ifndef _GRID_MAP_H
#define _GRID_MAP_H

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <random>
#include <nav_msgs/msg/odometry.hpp>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <tuple>
#include <visualization_msgs/msg/marker.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <plan_env/raycast.h>

#define logit(x) (log((x) / (1 - (x))))

using namespace std;

/**
 * @brief Eigen矩阵哈希函数模板，用于体素哈希索引
 * @tparam T Eigen矩阵类型
 * 
 * 为Eigen矩阵类型提供哈希函数，主要用于：
 * - 体素网格的快速索引
 * - 无序映射容器中的键值
 */
template <typename T>
struct matrix_hash : std::unary_function<T, size_t>
{
  /**
   * @brief 计算矩阵的哈希值
   * @param matrix 输入的Eigen矩阵
   * @return 计算得到的哈希值
   */
  std::size_t operator()(T const &matrix) const
  {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i)
    {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

/**
 * @brief 地图建构参数结构体
 * 
 * 包含所有地图建构相关的配置参数：
 * - 地图几何属性和边界
 * - 相机内参和传感器配置
 * - 光线投射和概率更新参数
 * - 可视化和性能监控设置
 */
struct MappingParameters
{
  /* map properties */
  Eigen::Vector3d map_origin_, map_size_;                   ///< 地图原点和尺寸
  Eigen::Vector3d map_min_boundary_, map_max_boundary_;     ///< 地图在位置坐标系下的边界
  Eigen::Vector3i map_voxel_num_;                           ///< 地图在索引坐标系下的体素数量
  Eigen::Vector3d local_update_range_;                      ///< 局部更新范围
  double resolution_, resolution_inv_;                       ///< 地图分辨率和倒数
  double obstacles_inflation_;                               ///< 障碍物膨胀半径
  string frame_id_;                                         ///< 坐标系ID
  int pose_type_;                                           ///< 位姿类型标识

  /* camera parameters */
  double cx_, cy_, fx_, fy_;                                ///< 相机内参：光心坐标和焦距

  /* time out */
  double odom_depth_timeout_;                               ///< 里程计深度数据超时阈值

  /* depth image projection filtering */
  double depth_filter_maxdist_, depth_filter_mindist_, depth_filter_tolerance_;  ///< 深度图像滤波参数
  int depth_filter_margin_;                                 ///< 深度滤波边缘裕度
  bool use_depth_filter_;                                   ///< 是否使用深度滤波
  double k_depth_scaling_factor_;                           ///< 深度缩放因子
  int skip_pixel_;                                          ///< 像素跳跃间隔

  /* raycasting */
  double p_hit_, p_miss_, p_min_, p_max_, p_occ_;          ///< 占据概率参数
  double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_,
      min_occupancy_log_;                                   ///< 占据概率的对数形式
  double min_ray_length_, max_ray_length_;                  ///< 光线投射的距离范围

  /* local map update and clear */
  int local_map_margin_;                                    ///< 局部地图更新边界

  /* visualization and computation time display */
  double visualization_truncate_height_, virtual_ceil_height_, ground_height_, virtual_ceil_yp_, virtual_ceil_yn_;  ///< 可视化高度参数
  bool show_occ_time_;                                      ///< 是否显示占据更新时间

  /* active mapping */
  double unknown_flag_;                                     ///< 未知区域标识值
};

/**
 * @brief 地图建构中间数据结构体
 * 
 * 存储地图融合过程中的中间数据：
 * - 占据网格缓冲区
 * - 相机位姿和深度图像数据
 * - 光线投射加速标志
 * - 性能统计信息
 */
struct MappingData
{
  // main map data, occupancy of each voxel and Euclidean distance
  std::vector<double> occupancy_buffer_;                    ///< 体素占据概率缓冲区
  std::vector<char> occupancy_buffer_inflate_;              ///< 膨胀后的占据状态缓冲区

  // camera position and pose data
  Eigen::Vector3d camera_pos_, last_camera_pos_;            ///< 当前和上一帧相机位置
  Eigen::Matrix3d camera_r_m_, last_camera_r_m_;            ///< 当前和上一帧相机旋转矩阵
  Eigen::Matrix4d cam2body_;                                ///< 相机到机体的变换矩阵

  // depth image data
  cv::Mat depth_image_, last_depth_image_;                  ///< 当前和上一帧深度图像
  int image_cnt_;                                           ///< 图像帧计数器

  // flags of map state
  bool occ_need_update_, local_updated_;                    ///< 占据更新和局部更新标志
  bool has_first_depth_;                                    ///< 是否接收到第一帧深度图像
  bool has_odom_, has_cloud_;                               ///< 是否有里程计和点云数据

  // odom_depth_timeout_
  rclcpp::Time last_occ_update_time_;                       ///< 上次占据更新时间
  bool flag_depth_odom_timeout_;                            ///< 深度里程计超时标志
  bool flag_use_depth_fusion;                               ///< 是否使用深度融合标志

  // depth image projected point cloud
  vector<Eigen::Vector3d> proj_points_;                     ///< 深度图像投影的点云
  int proj_points_cnt;                                      ///< 投影点数量

  // flag buffers for speeding up raycasting
  vector<short> count_hit_, count_hit_and_miss_;            ///< 光线命中计数缓冲区
  vector<char> flag_traverse_, flag_rayend_;                ///< 光线遍历和终点标志
  char raycast_num_;                                        ///< 光线投射编号
  queue<Eigen::Vector3i> cache_voxel_;                      ///< 体素缓存队列

  // range of updating grid
  Eigen::Vector3i local_bound_min_, local_bound_max_;       ///< 局部更新边界

  // computation time
  double fuse_time_, max_fuse_time_;                        ///< 融合时间统计
  int update_num_;                                          ///< 更新次数统计

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @brief 3D栅格占据地图类
 * 
 * 该类实现了完整的3D栅格占据地图功能：
 * - 深度图像与里程计数据的同步处理
 * - 基于概率的占据网格更新
 * - 光线投射算法进行空间推理
 * - 障碍物膨胀和碰撞检测
 * - 实时地图可视化和发布
 */
class GridMap
{
public:
  /**
   * @brief 默认构造函数
   */
  GridMap() {}
  
  /**
   * @brief 析构函数
   */
  ~GridMap() {}

  /**
   * @brief 位姿消息类型枚举
   */
  enum
  {
    POSE_STAMPED = 1,    ///< PoseStamped消息类型
    ODOMETRY = 2,        ///< Odometry消息类型
    INVALID_IDX = -10000 ///< 无效索引值
  };

  // occupancy map management
  
  /**
   * @brief 重置整个地图缓冲区
   * 
   * 清空所有占据概率数据，重新初始化地图状态
   */
  void resetBuffer();
  
  /**
   * @brief 重置指定区域的地图缓冲区
   * @param min 重置区域的最小边界
   * @param max 重置区域的最大边界
   */
  void resetBuffer(Eigen::Vector3d min, Eigen::Vector3d max);

  /**
   * @brief 将世界坐标转换为栅格索引
   * @param pos 世界坐标位置
   * @param id 输出的栅格索引
   */
  inline void posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id);
  
  /**
   * @brief 将栅格索引转换为世界坐标
   * @param id 栅格索引
   * @param pos 输出的世界坐标位置
   */
  inline void indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos);
  
  /**
   * @brief 将3D索引转换为1D地址
   * @param id 3D栅格索引
   * @return 对应的1D数组地址
   */
  inline int toAddress(const Eigen::Vector3i &id);
  
  /**
   * @brief 将3D坐标转换为1D地址
   * @param x X轴索引
   * @param y Y轴索引
   * @param z Z轴索引
   * @return 对应的1D数组地址
   */
  inline int toAddress(int &x, int &y, int &z);
  
  /**
   * @brief 检查世界坐标是否在地图范围内
   * @param pos 世界坐标位置
   * @return 在地图内返回true，否则返回false
   */
  inline bool isInMap(const Eigen::Vector3d &pos);
  
  /**
   * @brief 检查栅格索引是否在地图范围内
   * @param idx 栅格索引
   * @return 在地图内返回true，否则返回false
   */
  inline bool isInMap(const Eigen::Vector3i &idx);

  /**
   * @brief 设置指定位置的占据概率
   * @param pos 世界坐标位置
   * @param occ 占据概率值（默认为1.0）
   */
  inline void setOccupancy(Eigen::Vector3d pos, double occ = 1);
  
  /**
   * @brief 将指定位置设置为占据状态
   * @param pos 世界坐标位置
   */
  inline void setOccupied(Eigen::Vector3d pos);
  
  /**
   * @brief 获取指定位置的占据状态
   * @param pos 世界坐标位置
   * @return 占据状态：1为占据，0为自由，-1为超出边界
   */
  inline int getOccupancy(Eigen::Vector3d pos);
  
  /**
   * @brief 获取指定索引的占据状态
   * @param id 栅格索引
   * @return 占据状态：1为占据，0为自由，-1为超出边界
   */
  inline int getOccupancy(Eigen::Vector3i id);
  
  /**
   * @brief 获取指定位置的膨胀占据状态
   * @param pos 世界坐标位置
   * @return 膨胀占据状态：1为占据，0为自由，-1为超出边界
   */
  inline int getInflateOccupancy(Eigen::Vector3d pos);

  /**
   * @brief 将索引限制在地图边界内
   * @param id 输入输出的栅格索引，超出边界时会被修正
   */
  inline void boundIndex(Eigen::Vector3i &id);
  
  /**
   * @brief 检查指定索引是否为未知区域
   * @param id 栅格索引
   * @return 未知区域返回true，否则返回false
   */
  inline bool isUnknown(const Eigen::Vector3i &id);
  
  /**
   * @brief 检查指定位置是否为未知区域
   * @param pos 世界坐标位置
   * @return 未知区域返回true，否则返回false
   */
  inline bool isUnknown(const Eigen::Vector3d &pos);
  
  /**
   * @brief 检查指定索引是否为已知自由空间
   * @param id 栅格索引
   * @return 已知自由返回true，否则返回false
   */
  inline bool isKnownFree(const Eigen::Vector3i &id);
  
  /**
   * @brief 检查指定索引是否为已知占据区域
   * @param id 栅格索引
   * @return 已知占据返回true，否则返回false
   */
  inline bool isKnownOccupied(const Eigen::Vector3i &id);

  /**
   * @brief 初始化地图系统
   * @param node ROS2节点智能指针
   * 
   * 该函数完成以下初始化工作：
   * - 读取地图参数配置
   * - 设置订阅者和发布者
   * - 初始化地图缓冲区
   * - 配置传感器同步策略
   */
  void initMap(rclcpp::Node::SharedPtr node);

  /**
   * @brief 发布原始占据地图
   * 
   * 将当前占据网格发布为点云消息，用于可视化和调试
   */
  void publishMap();
  
  /**
   * @brief 发布膨胀后的占据地图
   * @param all_info 是否发布所有信息（默认为false）
   */
  void publishMapInflate(bool all_info = false);

  /**
   * @brief 发布深度图像信息
   * 
   * 将当前处理的深度图像发布，用于调试和监控
   */
  void publishDepth();

  /**
   * @brief 检查是否有深度观测数据
   * @return 有深度数据返回true，否则返回false
   */
  bool hasDepthObservation();
  
  /**
   * @brief 检查里程计数据是否有效
   * @return 里程计有效返回true，否则返回false
   */
  bool odomValid();
  
  /**
   * @brief 获取地图区域信息
   * @param ori 输出地图原点
   * @param size 输出地图尺寸
   */
  void getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size);
  
  /**
   * @brief 获取地图分辨率
   * @return 地图分辨率值
   */
  inline double getResolution();
  
  /**
   * @brief 获取地图原点
   * @return 地图原点坐标
   */
  Eigen::Vector3d getOrigin();
  
  /**
   * @brief 获取体素总数
   * @return 地图中的体素总数
   */
  int getVoxelNum();
  
  /**
   * @brief 获取深度里程计超时状态
   * @return 超时返回true，否则返回false
   */
  bool getOdomDepthTimeout() { return md_.flag_depth_odom_timeout_; }

  typedef std::shared_ptr<GridMap> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  MappingParameters mp_;  ///< 地图参数配置
  MappingData md_;        ///< 地图中间数据

  /**
   * @brief 深度图像和位姿同步回调函数
   * @param img 深度图像消息
   * @param pose 位姿消息
   * 
   * 处理同步的深度图像和位姿数据，更新相机状态
   */
  void depthPoseCallback(const sensor_msgs::msg::Image::ConstPtr &img,
                         const geometry_msgs::msg::PoseStamped::ConstPtr &pose);
  
  /**
   * @brief 外参标定回调函数
   * @param odom 里程计消息
   * 
   * 处理相机到机体的外参标定信息
   */
  void extrinsicCallback(const nav_msgs::msg::Odometry::ConstPtr &odom);
  
  /**
   * @brief 深度图像和里程计同步回调函数
   * @param img 深度图像消息
   * @param odom 里程计消息
   * 
   * 处理同步的深度图像和里程计数据，是主要的建图入口
   */
  void depthOdomCallback(const sensor_msgs::msg::Image::ConstPtr &img, const nav_msgs::msg::Odometry::ConstPtr &odom);
  
  /**
   * @brief 点云数据回调函数
   * @param img 点云消息
   * 
   * 处理独立的点云数据，用于地图更新
   */
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstPtr &img);
  
  /**
   * @brief 里程计数据回调函数
   * @param odom 里程计消息
   * 
   * 处理独立的里程计数据，更新机器人位姿
   */
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);

  /**
   * @brief 占据网格更新回调函数
   * 
   * 定时触发的地图更新函数，执行主要的建图流程：
   * - 深度图像投影
   * - 光线投射处理
   * - 局部地图清理和膨胀
   */
  void updateOccupancyCallback();
  
  /**
   * @brief 可视化回调函数
   * 
   * 定时发布地图可视化信息
   */
  void visCallback();

  /**
   * @brief 深度图像投影处理
   * 
   * 将深度图像投影到3D空间，生成观测点云：
   * - 深度图像滤波和预处理
   * - 相机模型投影计算
   * - 坐标系变换
   */
  void projectDepthImage();
  
  /**
   * @brief 光线投射处理
   * 
   * 执行光线投射算法更新占据概率：
   * - 从相机位置到观测点的射线追踪
   * - 概率占据更新
   * - 自由空间标记
   */
  void raycastProcess();
  
  /**
   * @brief 清理和膨胀局部地图
   * 
   * 对更新区域进行后处理：
   * - 清理边界区域
   * - 障碍物膨胀处理
   * - 安全距离计算
   */
  void clearAndInflateLocalMap();

  /**
   * @brief 膨胀指定点周围的体素
   * @param pt 中心点索引
   * @param step 膨胀步长
   * @param pts 输出的膨胀点集合
   * 
   * 对指定体素进行3D膨胀，生成安全边界
   */
  inline void inflatePoint(const Eigen::Vector3i &pt, int step, vector<Eigen::Vector3i> &pts);
  
  /**
   * @brief 设置缓存占据状态
   * @param pos 世界坐标位置
   * @param occ 占据状态值
   * @return 设置成功返回正值，失败返回负值
   * 
   * 高效的占据状态设置函数，使用缓存加速
   */
  int setCacheOccupancy(Eigen::Vector3d pos, int occ);
  
  /**
   * @brief 找到地图内距离指定点最近的点
   * @param pt 目标点
   * @param camera_pt 相机位置
   * @return 地图边界内的最近点
   * 
   * 用于处理超出地图边界的观测点
   */
  Eigen::Vector3d closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt);

  // 消息同步策略类型定义
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, nav_msgs::msg::Odometry>
      SyncPolicyImageOdom;                                      ///< 图像-里程计同步策略
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, geometry_msgs::msg::PoseStamped>
      SyncPolicyImagePose;                                      ///< 图像-位姿同步策略
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>> SynchronizerImagePose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImageOdom>> SynchronizerImageOdom;

  // ROS2接口
  rclcpp::Node::SharedPtr node_;                                ///< ROS2节点指针
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;           ///< 深度图像订阅者
  std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>> pose_sub_;    ///< 位姿订阅者
  std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_sub_;            ///< 里程计订阅者
  SynchronizerImagePose sync_image_pose_;                       ///< 图像-位姿同步器
  SynchronizerImageOdom sync_image_odom_;                       ///< 图像-里程计同步器

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr indep_cloud_sub_;            ///< 独立点云订阅者
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr indep_odom_sub_;                   ///< 独立里程计订阅者
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr extrinsic_sub_;                    ///< 外参订阅者

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;                       ///< 地图发布者
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_inf_pub_;                   ///< 膨胀地图发布者

  rclcpp::TimerBase::SharedPtr occ_timer_;                      ///< 占据更新定时器
  rclcpp::TimerBase::SharedPtr vis_timer_;                      ///< 可视化定时器

  // 随机数生成器
  uniform_real_distribution<double> rand_noise_;                ///< 均匀分布随机噪声
  normal_distribution<double> rand_noise2_;                     ///< 正态分布随机噪声
  default_random_engine eng_;                                   ///< 随机数引擎
};

/* ============================== definition of inline function
 * ============================== */

/**
 * @brief 将3D索引转换为1D地址的内联实现
 * @param id 3D栅格索引向量
 * @return 对应的1D数组地址
 * 
 * 使用行主序存储方式计算线性地址
 */
inline int GridMap::toAddress(const Eigen::Vector3i &id)
{
  return id(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + id(1) * mp_.map_voxel_num_(2) + id(2);
}

/**
 * @brief 将3D坐标转换为1D地址的内联实现
 * @param x X轴索引
 * @param y Y轴索引  
 * @param z Z轴索引
 * @return 对应的1D数组地址
 */
inline int GridMap::toAddress(int &x, int &y, int &z)
{
  return x * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + y * mp_.map_voxel_num_(2) + z;
}

/**
 * @brief 将索引限制在地图边界内的内联实现
 * @param id 输入输出的栅格索引，超出边界时会被修正到边界
 */
inline void GridMap::boundIndex(Eigen::Vector3i &id)
{
  Eigen::Vector3i id1;
  id1(0) = max(min(id(0), mp_.map_voxel_num_(0) - 1), 0);
  id1(1) = max(min(id(1), mp_.map_voxel_num_(1) - 1), 0);
  id1(2) = max(min(id(2), mp_.map_voxel_num_(2) - 1), 0);
  id = id1;
}

/**
 * @brief 检查指定索引是否为未知区域的内联实现
 * @param id 栅格索引
 * @return 未知区域返回true，否则返回false
 * 
 * 通过比较占据概率对数值与最小阈值判断未知状态
 */
inline bool GridMap::isUnknown(const Eigen::Vector3i &id)
{
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  return md_.occupancy_buffer_[toAddress(id1)] < mp_.clamp_min_log_ - 1e-3;
}

/**
 * @brief 检查指定位置是否为未知区域的内联实现
 * @param pos 世界坐标位置
 * @return 未知区域返回true，否则返回false
 */
inline bool GridMap::isUnknown(const Eigen::Vector3d &pos)
{
  Eigen::Vector3i idc;
  posToIndex(pos, idc);
  return isUnknown(idc);
}

/**
 * @brief 检查指定索引是否为已知自由空间的内联实现
 * @param id 栅格索引
 * @return 已知自由返回true，否则返回false
 * 
 * 同时检查占据概率和膨胀状态
 */
inline bool GridMap::isKnownFree(const Eigen::Vector3i &id)
{
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  int adr = toAddress(id1);

  // return md_.occupancy_buffer_[adr] >= mp_.clamp_min_log_ &&
  //     md_.occupancy_buffer_[adr] < mp_.min_occupancy_log_;
  return md_.occupancy_buffer_[adr] >= mp_.clamp_min_log_ && md_.occupancy_buffer_inflate_[adr] == 0;
}

/**
 * @brief 检查指定索引是否为已知占据区域的内联实现
 * @param id 栅格索引
 * @return 已知占据返回true，否则返回false
 * 
 * 通过膨胀缓冲区快速判断占据状态
 */
inline bool GridMap::isKnownOccupied(const Eigen::Vector3i &id)
{
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  int adr = toAddress(id1);

  return md_.occupancy_buffer_inflate_[adr] == 1;
}

/**
 * @brief 将指定位置设置为占据状态的内联实现
 * @param pos 世界坐标位置
 * 
 * 直接在膨胀缓冲区中标记为占据
 */
inline void GridMap::setOccupied(Eigen::Vector3d pos)
{
  if (!isInMap(pos))
    return;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  md_.occupancy_buffer_inflate_[id(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
                                id(1) * mp_.map_voxel_num_(2) + id(2)] = 1;
}

/**
 * @brief 设置指定位置的占据概率的内联实现
 * @param pos 世界坐标位置
 * @param occ 占据概率值（0或1）
 * 
 * 在原始占据缓冲区中设置概率值
 */
inline void GridMap::setOccupancy(Eigen::Vector3d pos, double occ)
{
  if (occ != 1 && occ != 0)
  {
    cout << "occ value error!" << endl;
    return;
  }

  if (!isInMap(pos))
    return;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  md_.occupancy_buffer_[toAddress(id)] = occ;
}

/**
 * @brief 获取指定位置的占据状态的内联实现
 * @param pos 世界坐标位置
 * @return 占据状态：1为占据，0为自由，-1为超出边界
 */
inline int GridMap::getOccupancy(Eigen::Vector3d pos)
{
  if (!isInMap(pos))
    return -1;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  return md_.occupancy_buffer_[toAddress(id)] > mp_.min_occupancy_log_ ? 1 : 0;
}

/**
 * @brief 获取指定位置的膨胀占据状态的内联实现
 * @param pos 世界坐标位置
 * @return 膨胀占据状态：1为占据，0为自由，-1为超出边界
 */
inline int GridMap::getInflateOccupancy(Eigen::Vector3d pos)
{
  if (!isInMap(pos))
    return -1;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  return int(md_.occupancy_buffer_inflate_[toAddress(id)]);
}

/**
 * @brief 获取指定索引的占据状态的内联实现
 * @param id 栅格索引
 * @return 占据状态：1为占据，0为自由，-1为超出边界
 */
inline int GridMap::getOccupancy(Eigen::Vector3i id)
{
  if (id(0) < 0 || id(0) >= mp_.map_voxel_num_(0) || id(1) < 0 || id(1) >= mp_.map_voxel_num_(1) ||
      id(2) < 0 || id(2) >= mp_.map_voxel_num_(2))
    return -1;

  return md_.occupancy_buffer_[toAddress(id)] > mp_.min_occupancy_log_ ? 1 : 0;
}

/**
 * @brief 检查世界坐标是否在地图范围内的内联实现
 * @param pos 世界坐标位置
 * @return 在地图内返回true，否则返回false
 * 
 * 检查坐标是否在地图的最小和最大边界内
 */
inline bool GridMap::isInMap(const Eigen::Vector3d &pos)
{
  if (pos(0) < mp_.map_min_boundary_(0) + 1e-4 || pos(1) < mp_.map_min_boundary_(1) + 1e-4 ||
      pos(2) < mp_.map_min_boundary_(2) + 1e-4)
  {
    // cout << "less than min range!" << endl;
    return false;
  }
  if (pos(0) > mp_.map_max_boundary_(0) - 1e-4 || pos(1) > mp_.map_max_boundary_(1) - 1e-4 ||
      pos(2) > mp_.map_max_boundary_(2) - 1e-4)
  {
    return false;
  }
  return true;
}

/**
 * @brief 检查栅格索引是否在地图范围内的内联实现
 * @param idx 栅格索引
 * @return 在地图内返回true，否则返回false
 */
inline bool GridMap::isInMap(const Eigen::Vector3i &idx)
{
  if (idx(0) < 0 || idx(1) < 0 || idx(2) < 0)
  {
    return false;
  }
  if (idx(0) > mp_.map_voxel_num_(0) - 1 || idx(1) > mp_.map_voxel_num_(1) - 1 ||
      idx(2) > mp_.map_voxel_num_(2) - 1)
  {
    return false;
  }
  return true;
}

/**
 * @brief 将世界坐标转换为栅格索引的内联实现
 * @param pos 世界坐标位置
 * @param id 输出的栅格索引
 * 
 * 使用地图原点和分辨率进行坐标变换
 */
inline void GridMap::posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id)
{
  for (int i = 0; i < 3; ++i)
    id(i) = floor((pos(i) - mp_.map_origin_(i)) * mp_.resolution_inv_);
}

/**
 * @brief 将栅格索引转换为世界坐标的内联实现
 * @param id 栅格索引
 * @param pos 输出的世界坐标位置
 * 
 * 返回体素中心点的世界坐标
 */
inline void GridMap::indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos)
{
  for (int i = 0; i < 3; ++i)
    pos(i) = (id(i) + 0.5) * mp_.resolution_ + mp_.map_origin_(i);
}

/**
 * @brief 膨胀指定点周围的体素的内联实现
 * @param pt 中心点索引
 * @param step 膨胀步长（立方体半径）
 * @param pts 输出的膨胀点集合
 * 
 * 对指定体素进行立方体膨胀，生成周围的所有体素点
 */
inline void GridMap::inflatePoint(const Eigen::Vector3i &pt, int step, vector<Eigen::Vector3i> &pts)
{
  int num = 0;
  /* ---------- + shape inflate ---------- */
  // for (int x = -step; x <= step; ++x)
  // {
  //   if (x == 0)
  //     continue;
  //   pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1), pt(2));
  // }
  // for (int y = -step; y <= step; ++y)
  // {
  //   if (y == 0)
  //     continue;
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1) + y, pt(2));
  // }
  // for (int z = -1; z <= 1; ++z)
  // {
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1), pt(2) + z);
  // }

  /* ---------- all inflate ---------- */
  for (int x = -step; x <= step; ++x)
    for (int y = -step; y <= step; ++y)
      for (int z = -step; z <= step; ++z)
      {
        pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1) + y, pt(2) + z);
      }
}

/**
 * @brief 获取地图分辨率的内联实现
 * @return 地图分辨率值（米/体素）
 */
inline double GridMap::getResolution() { return mp_.resolution_; }

#endif