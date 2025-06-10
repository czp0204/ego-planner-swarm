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
 * @file linear_obj_model.hpp
 * @brief EGO-Planner中的线性动态障碍物模型
 * @author Boyu Zhou, Aerial Robotics Group, HKUST
 * @date 2019
 * 
 * 该文件实现了用于仿真的线性动态障碍物模型：
 * - 支持基于速度或加速度的控制输入模式
 * - 实现边界碰撞检测和反弹处理
 * - 提供障碍物间的碰撞检测和处理
 * - 支持三重积分器动力学模型
 * - 用于动态环境的仿真测试
 */

#ifndef _LINEAR_OBJ_MODEL_H_
#define _LINEAR_OBJ_MODEL_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>

/**
 * @brief 线性动态障碍物模型类
 * 
 * 该类实现了一个简化的动态障碍物仿真模型：
 * - 支持位置、速度、加速度的状态更新
 * - 提供两种控制模式：速度控制和加速度控制
 * - 实现边界约束和碰撞处理逻辑
 * - 支持障碍物间的交互碰撞检测
 * - 维护障碍物的几何属性和可视化信息
 */
class LinearObjModel {
private:
  bool last_out_bound_{false};  ///< 上一时刻是否超出边界的标志
  int input_type_;              ///< 控制输入类型：1-速度控制，2-加速度控制
  
public:
  /**
   * @brief 默认构造函数
   */
  LinearObjModel(/* args */);
  
  /**
   * @brief 析构函数
   */
  ~LinearObjModel();

  /**
   * @brief 初始化障碍物模型
   * @param p 初始位置
   * @param v 初始速度
   * @param a 初始加速度
   * @param yaw 初始偏航角
   * @param yaw_dot 偏航角速度
   * @param color 显示颜色RGB值
   * @param scale 障碍物尺寸[长,宽,高]
   * @param input_type 控制输入类型（1:速度，2:加速度）
   * 
   * 设置障碍物的初始状态和属性
   */
  void initialize(Eigen::Vector3d p, Eigen::Vector3d v, Eigen::Vector3d a, double yaw, double yaw_dot,
                  Eigen::Vector3d color, Eigen::Vector3d scale, int input_type);

  /**
   * @brief 设置运动约束限制
   * @param bound 空间边界约束[x_max, y_max, z_max]
   * @param vel 速度约束[v_min, v_max]
   * @param acc 加速度约束[a_min, a_max]
   * 
   * 定义障碍物的运动边界和动力学限制
   */
  void setLimits(Eigen::Vector3d bound, Eigen::Vector2d vel, Eigen::Vector2d acc);

  /**
   * @brief 更新障碍物状态
   * @param dt 时间步长
   * 
   * 根据当前控制输入和动力学模型更新障碍物状态：
   * - 加速度控制模式：使用三重积分器模型
   * - 速度控制模式：使用运动学模型
   * - 处理边界碰撞和反弹
   * - 限制速度在允许范围内
   */
  void update(double dt);

  /**
   * @brief 障碍物间碰撞检测和处理
   * @param obj1 第一个障碍物对象
   * @param obj2 第二个障碍物对象
   * @return 发生碰撞返回true，否则返回false
   * 
   * 检测两个障碍物是否发生碰撞：
   * - 基于包围盒的碰撞检测
   * - 计算碰撞深度和分离方向
   * - 自动处理碰撞响应和位置分离
   * - 更新碰撞后的速度
   */
  static bool collide(LinearObjModel& obj1, LinearObjModel& obj2);

  /**
   * @brief 设置速度控制输入
   * @param vel 目标速度向量
   * 
   * 在速度控制模式下设置期望速度
   */
  void setInput(Eigen::Vector3d vel) {
    vel_ = vel;
  }

  /**
   * @brief 设置偏航角速度
   * @param yaw_dot 偏航角速度（弧度/秒）
   */
  void setYawDot(double yaw_dot) {
    yaw_dot_ = yaw_dot;
  }

  /**
   * @brief 获取当前位置
   * @return 3D位置坐标
   */
  Eigen::Vector3d getPosition() {
    return pos_;
  }
  
  /**
   * @brief 设置位置
   * @param pos 新的位置坐标
   */
  void setPosition(Eigen::Vector3d pos) {
    pos_ = pos;
  }

  /**
   * @brief 获取当前速度
   * @return 3D速度向量
   */
  Eigen::Vector3d getVelocity() {
    return vel_;
  }

  /**
   * @brief 设置速度
   * @param x X轴速度分量
   * @param y Y轴速度分量
   * @param z Z轴速度分量
   */
  void setVelocity(double x, double y, double z) {
    vel_ = Eigen::Vector3d(x, y, z);
  }

  /**
   * @brief 获取显示颜色
   * @return RGB颜色值
   */
  Eigen::Vector3d getColor() {
    return color_;
  }
  
  /**
   * @brief 获取障碍物尺寸
   * @return 3D尺寸向量[长,宽,高]
   */
  Eigen::Vector3d getScale() {
    return scale_;
  }

  /**
   * @brief 获取当前偏航角
   * @return 偏航角（弧度）
   */
  double getYaw() {
    return yaw_;
  }

private:
  Eigen::Vector3d pos_, vel_, acc_;  ///< 位置、速度、加速度状态
  Eigen::Vector3d color_, scale_;    ///< 颜色和尺寸属性
  double yaw_, yaw_dot_;             ///< 偏航角和偏航角速度

  Eigen::Vector3d bound_;            ///< 空间边界约束
  Eigen::Vector2d limit_v_, limit_a_; ///< 速度和加速度限制
};

/**
 * @brief 构造函数实现
 */
LinearObjModel::LinearObjModel(/* args */) {
}

/**
 * @brief 析构函数实现
 */
LinearObjModel::~LinearObjModel() {
}

/**
 * @brief 初始化函数实现
 */
void LinearObjModel::initialize(Eigen::Vector3d p, Eigen::Vector3d v, Eigen::Vector3d a, double yaw,
                                double yaw_dot, Eigen::Vector3d color, Eigen::Vector3d scale, int input_type) {
  pos_ = p;
  vel_ = v;
  acc_ = a;
  color_ = color;
  scale_ = scale;
  input_type_ = input_type;

  yaw_ = yaw;
  yaw_dot_ = yaw_dot;
}

/**
 * @brief 设置约束限制实现
 */
void LinearObjModel::setLimits(Eigen::Vector3d bound, Eigen::Vector2d vel, Eigen::Vector2d acc) {
  bound_ = bound;
  limit_v_ = vel;
  limit_a_ = acc;
}

/**
 * @brief 状态更新函数实现
 */
void LinearObjModel::update(double dt) {
  Eigen::Vector3d p0, v0, a0;
  p0 = pos_, v0 = vel_, a0 = acc_;
  //std::cout << v0.transpose() << std::endl;

  /* ---------- use acc as input ---------- */
  if ( input_type_ == 2 )
  {
    vel_ = v0 + acc_ * dt;
    for (int i = 0; i < 3; ++i)
    {
      if (vel_(i) > 0) vel_(i) = std::max(limit_v_(0), std::min(vel_(i),
      limit_v_(1)));
      if (vel_(i) <= 0) vel_(i) = std::max(-limit_v_(1), std::min(vel_(i),
      -limit_v_(0)));
    }

    pos_ = p0 + v0 * dt + 0.5 * acc_ * pow(dt, 2);

    /* ---------- reflect acc when collide with bound ---------- */
    if ( pos_(0) <= bound_(0) && pos_(0) >= -bound_(0) &&
        pos_(1) <= bound_(1) && pos_(1) >= -bound_(1) &&
        pos_(2) <= bound_(2) && pos_(2) >= 0
      )
    {
      last_out_bound_ = false;
    }
    else if ( !last_out_bound_ )
    {
      last_out_bound_ = true;

      // if ( pos_(0) > bound_(0) || pos_(0) < -bound_(0) ) acc_(0) = -acc_(0);
      // if ( pos_(1) > bound_(1) || pos_(1) < -bound_(1) ) acc_(1) = -acc_(1);
      // if ( pos_(2) > bound_(2) || pos_(2) < -bound_(2) ) acc_(2) = -acc_(2);
      acc_ = -acc_;
      //ROS_ERROR("AAAAAAAAAAAAAAAAAAa");
    }
  }
  // for (int i = 0; i < 2; ++i)
  // {
  //   pos_(i) = std::min(pos_(i), bound_(i));
  //   pos_(i) = std::max(pos_(i), -bound_(i));
  // }
  // pos_(2) = std::min(pos_(2), bound_(2));
  // pos_(2) = std::max(pos_(2), 0.0);

  /* ---------- use vel as input ---------- */
  else if ( input_type_ == 1 )
  {
    pos_ = p0 + v0 * dt;
    for (int i = 0; i < 2; ++i) {
      pos_(i) = std::min(pos_(i), bound_(i));
      pos_(i) = std::max(pos_(i), -bound_(i));
    }
    pos_(2) = std::min(pos_(2), bound_(2));
    pos_(2) = std::max(pos_(2), 0.0);

    yaw_ += yaw_dot_ * dt;

    const double PI = 3.1415926;
    if (yaw_ > 2 * PI) yaw_ -= 2 * PI;

    const double tol = 0.1;
    if (pos_(0) > bound_(0) - tol) {
      pos_(0) = bound_(0) - tol;
      vel_(0) = -vel_(0);
    }
    if (pos_(0) < -bound_(0) + tol) {
      pos_(0) = -bound_(0) + tol;
      vel_(0) = -vel_(0);
    }

    if (pos_(1) > bound_(1) - tol) {
      pos_(1) = bound_(1) - tol;
      vel_(1) = -vel_(1);
    }
    if (pos_(1) < -bound_(1) + tol) {
      pos_(1) = -bound_(1) + tol;
      vel_(1) = -vel_(1);
    }

    if (pos_(2) > bound_(2) - tol) {
      pos_(2) = bound_(2) - tol;
      vel_(2) = -vel_(2);
    }
    if (pos_(2) < tol) {
      pos_(2) = tol;
      vel_(2) = -vel_(2);
    }
  }

  // /* ---------- reflect when collide with bound ---------- */


  //std::cout << pos_.transpose() << "  " << bound_.transpose() << std::endl;
}

/**
 * @brief 碰撞检测和处理函数实现
 */
bool LinearObjModel::collide(LinearObjModel& obj1, LinearObjModel& obj2) {
  Eigen::Vector3d pos1, pos2, vel1, vel2, scale1, scale2;
  pos1 = obj1.getPosition();
  vel1 = obj1.getVelocity();
  scale1 = obj1.getScale();

  pos2 = obj2.getPosition();
  vel2 = obj2.getVelocity();
  scale2 = obj2.getScale();

  /* ---------- collide ---------- */
  bool collide = fabs(pos1(0) - pos2(0)) < 0.5 * (scale1(0) + scale2(0)) &&
      fabs(pos1(1) - pos2(1)) < 0.5 * (scale1(1) + scale2(1)) &&
      fabs(pos1(2) - pos2(2)) < 0.5 * (scale1(2) + scale2(2));

  if (collide) {
    double tol[3];
    tol[0] = 0.5 * (scale1(0) + scale2(0)) - fabs(pos1(0) - pos2(0));
    tol[1] = 0.5 * (scale1(1) + scale2(1)) - fabs(pos1(1) - pos2(1));
    tol[2] = 0.5 * (scale1(2) + scale2(2)) - fabs(pos1(2) - pos2(2));

    for (int i = 0; i < 3; ++i) {
      if (tol[i] < tol[(i + 1) % 3] && tol[i] < tol[(i + 2) % 3]) {
        vel1(i) = -vel1(i);
        vel2(i) = -vel2(i);
        obj1.setVelocity(vel1(0), vel1(1), vel1(2));
        obj2.setVelocity(vel2(0), vel2(1), vel2(2));

        if (pos1(i) >= pos2(i)) {
          pos1(i) += tol[i];
          pos2(i) -= tol[i];
        } else {
          pos1(i) -= tol[i];
          pos2(i) += tol[i];
        }
        obj1.setPosition(pos1);
        obj2.setPosition(pos2);

        break;
      }
    }

    return true;
  } else {
    return false;
  }
}

#endif