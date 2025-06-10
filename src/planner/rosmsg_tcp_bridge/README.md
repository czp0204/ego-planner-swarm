# ROS消息TCP/UDP桥接器 (rosmsg_tcp_bridge)

## 概述

`rosmsg_tcp_bridge` 是EGO-Planner-Swarm系统中负责多机器人间通信的核心模块。该模块实现了ROS2消息与TCP/UDP网络协议之间的桥接，使得不同机器人之间能够通过网络进行实时的轨迹同步、里程计信息共享和紧急停止信号传递。

## 主要功能

### 1. 消息类型支持
- **轨迹消息 (MultiBsplines/Bspline)**: 多机器人轨迹同步
- **里程计消息 (Odometry)**: 机器人位置和速度信息广播
- **紧急停止消息 (Empty)**: 安全停止信号传递

### 2. 通信协议
- **TCP连接**: 用于可靠的轨迹数据传输
- **UDP广播**: 用于高频率的里程计和紧急信号传输

### 3. 网络拓扑
- 支持环形拓扑结构的多机器人网络
- 每个机器人同时作为客户端和服务器
- 自动建立与相邻机器人的连接

## 核心组件

### 消息序列化/反序列化
```cpp
// 支持的消息类型
enum MESSAGE_TYPE {
  ODOM = 888,      // 里程计消息
  MULTI_TRAJ,      // 多轨迹消息
  ONE_TRAJ,        // 单轨迹消息  
  STOP             // 停止消息
}
```

### 网络连接管理
- `connect_to_next_drone()`: 建立到下一个机器人的TCP连接
- `wait_connection_from_previous_drone()`: 等待上一个机器人的TCP连接
- `init_broadcast()`: 初始化UDP广播
- `udp_bind_to_port()`: 绑定UDP监听端口

### 多线程处理
- **主线程**: ROS2节点运行和消息处理
- **TCP接收线程**: 处理来自其他机器人的TCP消息
- **UDP接收线程**: 处理UDP广播消息

## 配置参数

| 参数名 | 类型 | 默认值 | 描述 |
|--------|------|--------|------|
| `drone_id` | int | - | 机器人ID |
| `tcp_ip` | string | - | TCP连接目标IP |
| `udp_ip` | string | - | UDP广播IP |
| `odom_broadcast_freq` | double | 10.0 | 里程计广播频率(Hz) |

## 通信流程

### 1. 启动阶段
1. 读取机器人ID和网络配置
2. 建立TCP连接到下一个机器人
3. 等待来自上一个机器人的TCP连接
4. 初始化UDP广播和接收

### 2. 运行阶段
- **轨迹同步**: 通过TCP可靠传输B样条轨迹
- **状态广播**: 通过UDP高频广播里程计信息
- **安全机制**: 紧急停止信号的实时传递

### 3. 消息处理
```cpp
// TCP消息处理 (可靠传输)
void multitraj_sub_tcp_cb()  // 多轨迹消息

// UDP消息处理 (高频广播)
void odom_sub_udp_cb()       // 里程计消息
void emergency_stop_sub_udp_cb()  // 紧急停止
void one_traj_sub_udp_cb()   // 单轨迹消息
```

## 依赖项

- **ROS2依赖**: rclcpp, std_msgs, geometry_msgs, nav_msgs
- **自定义消息**: traj_utils (Bspline, MultiBsplines)
- **系统库**: Boost.Thread, Socket API
- **数学库**: Eigen3

## 网络架构

```
机器人0 ←→ 机器人1 ←→ 机器人2 ←→ ... ←→ 机器人N ←→ 机器人0
   ↓           ↓           ↓              ↓           ↓
 UDP广播    UDP广播    UDP广播         UDP广播    UDP广播
   ↓           ↓           ↓              ↓           ↓
      所有机器人都能接收到广播消息
```

## 使用方法

### 1. 编译
```bash
colcon build --packages-select rosmsg_tcp_bridge
```

### 2. 启动节点
```bash
ros2 run rosmsg_tcp_bridge bridge_node \
  --ros-args \
  -p drone_id:=0 \
  -p tcp_ip:="192.168.1.101" \
  -p udp_ip:="192.168.1.255"
```

### 3. 多机器人启动
每个机器人需要配置不同的ID和相应的网络参数，形成完整的通信网络。

## 故障处理

### 常见问题
1. **TCP连接失败**: 检查网络连通性和防火墙设置
2. **UDP消息丢失**: 正常现象，系统设计为容忍UDP丢包
3. **消息序列化错误**: 检查消息版本兼容性

### 调试方法
- 使用 `netstat` 检查端口占用
- 使用 `wireshark` 监控网络流量
- 查看ROS2日志输出

## 性能特点

- **低延迟**: UDP广播实现毫秒级通信
- **可靠性**: TCP保证重要轨迹数据不丢失
- **可扩展**: 支持任意数量机器人组网
- **容错性**: 单点故障不影响整体系统运行 