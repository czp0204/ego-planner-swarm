/**
 * @file raycast.h
 * @brief 3D空间中的光线投射算法实现
 * @author EGO-Planner团队
 * @date 2024
 * 
 * 该文件实现了用于栅格地图的3D光线投射算法：
 * - 基于Bresenham算法的3D扩展
 * - 高效的体素遍历计算
 * - 支持起始点到终点的完整射线追踪
 * - 用于占据网格的光线投射更新
 */
#ifndef RAYCAST_H_
#define RAYCAST_H_

#include <Eigen/Eigen>
#include <vector>

/**
 * @brief 计算数值的符号函数
 * @param x 输入数值
 * @return 正数返回1，负数返回-1，零返回0
 * 
 * 用于确定射线方向的辅助函数
 */
double signum(double x);

/**
 * @brief 计算模运算
 * @param value 被除数
 * @param modulus 除数（模数）
 * @return 模运算结果
 * 
 * 处理边界条件的辅助函数
 */
double mod(double value, double modulus);

/**
 * @brief 计算到下一个整数边界的距离
 * @param s 当前位置
 * @param ds 方向增量
 * @return 到下一个整数边界的距离
 * 
 * 用于3D DDA算法的核心函数，计算射线到达下一个体素边界的参数
 */
double intbound(double s, double ds);

/**
 * @brief 3D光线投射函数（固定数组版本）
 * @param start 射线起始点世界坐标
 * @param end 射线终点世界坐标  
 * @param min 栅格地图最小边界
 * @param max 栅格地图最大边界
 * @param output_points_cnt 输出穿过的体素点数量
 * @param output 输出的体素点数组（需预分配足够空间）
 * 
 * 计算从起始点到终点射线穿过的所有体素中心点坐标
 * 使用3D DDA算法进行高效计算
 */
void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& min,
             const Eigen::Vector3d& max, int& output_points_cnt, Eigen::Vector3d* output);

/**
 * @brief 3D光线投射函数（动态数组版本）
 * @param start 射线起始点世界坐标
 * @param end 射线终点世界坐标
 * @param min 栅格地图最小边界
 * @param max 栅格地图最大边界
 * @param output 输出的体素点向量（自动调整大小）
 * 
 * 与固定数组版本功能相同，但使用std::vector自动管理内存
 */
void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& min,
             const Eigen::Vector3d& max, std::vector<Eigen::Vector3d>* output);

/**
 * @brief 增量式光线投射器类
 * 
 * 提供逐步遍历射线路径的功能：
 * - 支持初始化射线起终点
 * - 逐步返回射线路径上的下一个体素
 * - 高效的内存使用和计算
 * - 适用于需要中途停止或条件判断的场景
 */
class RayCaster {
private:
  /* data */
  Eigen::Vector3d start_;      ///< 射线起始点
  Eigen::Vector3d end_;        ///< 射线终点
  Eigen::Vector3d direction_;  ///< 射线方向向量
  Eigen::Vector3d min_;        ///< 地图最小边界
  Eigen::Vector3d max_;        ///< 地图最大边界
  int x_;                      ///< 当前X轴体素索引
  int y_;                      ///< 当前Y轴体素索引
  int z_;                      ///< 当前Z轴体素索引
  int endX_;                   ///< 终点X轴体素索引
  int endY_;                   ///< 终点Y轴体素索引
  int endZ_;                   ///< 终点Z轴体素索引
  double maxDist_;             ///< 射线最大长度
  double dx_;                  ///< X轴方向增量
  double dy_;                  ///< Y轴方向增量
  double dz_;                  ///< Z轴方向增量
  int stepX_;                  ///< X轴步进方向（+1或-1）
  int stepY_;                  ///< Y轴步进方向（+1或-1）
  int stepZ_;                  ///< Z轴步进方向（+1或-1）
  double tMaxX_;               ///< X轴下次边界交点的t参数
  double tMaxY_;               ///< Y轴下次边界交点的t参数
  double tMaxZ_;               ///< Z轴下次边界交点的t参数
  double tDeltaX_;             ///< X轴t参数增量
  double tDeltaY_;             ///< Y轴t参数增量
  double tDeltaZ_;             ///< Z轴t参数增量
  double dist_;                ///< 当前已遍历距离

  int step_num_;               ///< 步数计数器

public:
  /**
   * @brief 默认构造函数
   */
  RayCaster(/* args */) {
  }
  
  /**
   * @brief 析构函数
   */
  ~RayCaster() {
  }

  /**
   * @brief 设置射线起终点并初始化投射器
   * @param start 射线起始点世界坐标
   * @param end 射线终点世界坐标
   * @return 设置成功返回true，失败返回false
   * 
   * 该函数完成以下初始化工作：
   * - 计算射线方向和长度
   * - 初始化3D DDA算法参数
   * - 设置当前位置为起始体素
   * - 计算各轴的步进参数
   */
  bool setInput(const Eigen::Vector3d& start,
                const Eigen::Vector3d& end /* , const Eigen::Vector3d& min,
                const Eigen::Vector3d& max */);

  /**
   * @brief 执行一步射线投射，获取下一个体素点
   * @param ray_pt 输出下一个体素点的世界坐标
   * @return 还有下一个点返回true，已到达终点返回false
   * 
   * 该函数使用3D DDA算法计算射线路径上的下一个体素：
   * - 比较各轴的t参数确定下次跨越的边界
   * - 更新当前体素索引
   * - 返回体素中心点的世界坐标
   * - 检查是否已到达射线终点
   */
  bool step(Eigen::Vector3d& ray_pt);
};

#endif  // RAYCAST_H_