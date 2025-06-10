/**
 * This file is part of Fast-Planner.
 *
 * Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
 * Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
 * for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
 * If you use this code, please cite the respective publications as
 * listed on the above website.
 *
 * Fast-Planner is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Fast-Planner is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file obj_predictor.h
 * @brief EGO-Planner中的动态障碍物预测模块
 * @author Boyu Zhou, Aerial Robotics Group, HKUST
 * @date 2019
 * 
 * 该文件实现了动态障碍物的轨迹预测功能：
 * - 订阅并记录动态障碍物的历史位置信息
 * - 基于多项式拟合进行轨迹预测
 * - 支持恒定速度模型的预测
 * - 提供预测轨迹的实时查询接口
 * - 与路径规划器共享预测数据
 */

#ifndef _OBJ_PREDICTOR_H_
#define _OBJ_PREDICTOR_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <list>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

using std::cout;
using std::endl;
using std::list;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace fast_planner
{
  class PolynomialPrediction;
  typedef shared_ptr<vector<PolynomialPrediction>> ObjPrediction;  ///< 障碍物预测轨迹智能指针类型
  typedef shared_ptr<vector<Eigen::Vector3d>> ObjScale;            ///< 障碍物尺寸智能指针类型

  /**
   * @brief 多项式轨迹预测类
   * 
   * 使用5次多项式对动态障碍物的未来轨迹进行建模：
   * - 支持XYZ三个维度的独立多项式拟合
   * - 提供时间区间内的轨迹评估功能
   * - 支持恒定速度模型的简化预测
   * - 用于平滑的轨迹插值计算
   */
  class PolynomialPrediction
  {
  private:
    vector<Eigen::Matrix<double, 6, 1>> polys;  ///< 三个维度的5次多项式系数 [x, y, z]
    double t1, t2;                              ///< 预测时间区间的起始和结束时间
    rclcpp::Time global_start_time_;            ///< 全局起始时间基准

  public:
    /**
     * @brief 默认构造函数
     */
    PolynomialPrediction(/* args */)
    {
    }
    
    /**
     * @brief 析构函数
     */
    ~PolynomialPrediction()
    {
    }

    /**
     * @brief 设置多项式系数
     * @param pls 三个维度的多项式系数向量
     * 
     * 每个维度使用6个系数表示5次多项式：a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
     */
    void setPolynomial(vector<Eigen::Matrix<double, 6, 1>> &pls)
    {
      polys = pls;
    }
    
    /**
     * @brief 设置预测时间区间
     * @param t1 起始时间
     * @param t2 结束时间
     */
    void setTime(double t1, double t2)
    {
      this->t1 = t1;
      this->t2 = t2;
    }
    
    /**
     * @brief 设置全局起始时间基准
     * @param global_start_time ROS时间戳
     */
    void setGlobalStartTime(rclcpp::Time global_start_time)
    {
      global_start_time_ = global_start_time;
    }

    /**
     * @brief 检查多项式是否有效
     * @return 有效返回true（需要3个维度的多项式），否则返回false
     */
    bool valid()
    {
      return polys.size() == 3;
    }

    /**
     * @brief 在指定时间评估多项式轨迹位置
     * @param t 查询时间（需在[t1, t2]区间内）
     * @return 预测的3D位置坐标
     * 
     * 使用5次多项式计算轨迹上指定时刻的位置
     */
    Eigen::Vector3d evaluate(double t)
    {
      Eigen::Matrix<double, 6, 1> tv;
      tv << 1.0, pow(t, 1), pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5);

      Eigen::Vector3d pt;
      pt(0) = tv.dot(polys[0]), pt(1) = tv.dot(polys[1]), pt(2) = tv.dot(polys[2]);

      return pt;
    }

    /**
     * @brief 使用恒定速度模型评估轨迹位置
     * @param t 查询时间
     * @return 预测的3D位置坐标
     * 
     * 使用线性模型（恒定速度）进行轨迹预测，计算复杂度更低
     */
    Eigen::Vector3d evaluateConstVel(double t)
    {
      Eigen::Matrix<double, 2, 1> tv;
      tv << 1.0, pow(t - global_start_time_.seconds(), 1);

      // cout << t-global_start_time_.toSec() << endl;

      Eigen::Vector3d pt;
      pt(0) = tv.dot(polys[0].head(2)), pt(1) = tv.dot(polys[1].head(2)), pt(2) = tv.dot(polys[2].head(2));

      return pt;
    }
  };

  /**
   * @brief 障碍物历史轨迹记录类
   * 
   * 订阅并维护动态障碍物的历史位置信息：
   * - 缓存指定数量的历史位置点
   * - 支持数据抽样以减少计算量
   * - 提供历史数据的查询接口
   * - 自动管理缓存队列大小
   */
  class ObjHistory
  {
  public:
    int skip_num_;                  ///< 数据抽样间隔（每skip_num_个数据点取一个）
    int queue_size_;                ///< 历史数据队列最大长度
    rclcpp::Time global_start_time_; ///< 全局起始时间基准

    /**
     * @brief 默认构造函数
     */
    ObjHistory()
    {
    }
    
    /**
     * @brief 析构函数
     */
    ~ObjHistory()
    {
    }

    /**
     * @brief 初始化历史记录器
     * @param id 障碍物ID
     * @param skip_num 数据抽样间隔
     * @param queue_size 历史队列大小
     * @param global_start_time 全局起始时间
     */
    void init(int id, int skip_num, int queue_size, rclcpp::Time global_start_time);

    /**
     * @brief 位姿回调函数
     * @param msg 接收到的位姿消息
     * 
     * 处理订阅的障碍物位姿信息，更新历史轨迹缓存
     */
    void poseCallback(const geometry_msgs::msg::PoseStamped::ConstPtr &msg);

    /**
     * @brief 清空历史数据
     */
    void clear()
    {
      history_.clear();
    }

    /**
     * @brief 获取历史轨迹数据
     * @param his 输出的历史轨迹列表，每个元素为[x, y, z, t]
     */
    void getHistory(list<Eigen::Vector4d> &his)
    {
      his = history_;
    }

  private:
    list<Eigen::Vector4d> history_; ///< 历史轨迹数据，格式为[x, y, z, t]
    int skip_;                      ///< 当前抽样计数器
    int obj_idx_;                   ///< 障碍物索引
    Eigen::Vector3d scale_;         ///< 障碍物尺寸信息
  };

  /**
   * @brief 动态障碍物轨迹预测器类
   * 
   * 主要的预测模块，集成了多个障碍物的轨迹预测功能：
   * - 管理多个障碍物的历史数据记录
   * - 执行基于多项式拟合的轨迹预测
   * - 提供实时的轨迹查询服务
   * - 与路径规划模块共享预测结果
   * - 支持多种预测算法（多项式拟合、恒定速度）
   */
  class ObjPredictor : public rclcpp::Node
  {
  private:
    int obj_num_;                   ///< 跟踪的障碍物数量
    double lambda_;                 ///< 多项式拟合的正则化参数
    double predict_rate_;           ///< 预测更新频率

    vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_subs_;  ///< 位姿订阅者数组
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr marker_sub_;         ///< 标记订阅者
    rclcpp::TimerBase::SharedPtr predict_timer_;                                          ///< 预测定时器
    vector<std::shared_ptr<ObjHistory>> obj_histories_;                                   ///< 障碍物历史记录器数组

    /* share data with planner */
    ObjPrediction predict_trajs_;   ///< 与规划器共享的预测轨迹数据
    ObjScale obj_scale_;            ///< 与规划器共享的障碍物尺寸数据
    vector<bool> scale_init_;       ///< 障碍物尺寸初始化标志

    /**
     * @brief 标记消息回调函数
     * @param msg 接收到的可视化标记消息
     * 
     * 处理障碍物的可视化信息，提取尺寸等属性
     */
    void markerCallback(const visualization_msgs::msg::Marker::ConstPtr &msg);

    /**
     * @brief 预测定时器回调函数
     * 
     * 定期执行轨迹预测计算，更新预测结果
     */
    void predictCallback();
    
    /**
     * @brief 执行多项式拟合预测
     * 
     * 使用历史轨迹数据进行多项式拟合，生成平滑的预测轨迹
     */
    void predictPolyFit();
    
    /**
     * @brief 执行恒定速度预测
     * 
     * 使用简单的线性外推模型进行轨迹预测
     */
    void predictConstVel();

  public:
    /**
     * @brief 构造函数
     * @param node_name ROS2节点名称
     */
    ObjPredictor(const std::string &node_name = "obj_predictor")
        : Node(node_name) {}

    /**
     * @brief 析构函数
     */
    ~ObjPredictor() {}

    /**
     * @brief 初始化预测器
     * 
     * 完成以下初始化工作：
     * - 读取配置参数
     * - 设置订阅者和定时器
     * - 初始化障碍物历史记录器
     * - 配置预测算法参数
     */
    void init();

    /**
     * @brief 获取预测轨迹数据
     * @return 所有障碍物的预测轨迹智能指针
     */
    ObjPrediction getPredictionTraj();
    
    /**
     * @brief 获取障碍物尺寸数据
     * @return 所有障碍物的尺寸信息智能指针
     */
    ObjScale getObjScale();
    
    /**
     * @brief 获取障碍物数量
     * @return 当前跟踪的障碍物总数
     */
    int getObjNums() { return obj_num_; }

    /**
     * @brief 评估指定障碍物在指定时间的多项式预测位置
     * @param obs_id 障碍物ID
     * @param time 查询时间
     * @return 预测的3D位置坐标
     */
    Eigen::Vector3d evaluatePoly(int obs_id, double time);
    
    /**
     * @brief 评估指定障碍物在指定时间的恒定速度预测位置
     * @param obs_id 障碍物ID
     * @param time 查询时间
     * @return 预测的3D位置坐标
     */
    Eigen::Vector3d evaluateConstVel(int obs_id, double time);

    typedef std::shared_ptr<ObjPredictor> Ptr;  ///< 智能指针类型定义
  };

} // namespace fast_planner

#endif