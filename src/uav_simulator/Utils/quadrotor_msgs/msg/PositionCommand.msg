std_msgs/Header header
geometry_msgs/Point position          # 位置 (x, y, z)
geometry_msgs/Vector3 velocity        # 速度 (vx, vy, vz)  
geometry_msgs/Vector3 acceleration    # 加速度 (ax, ay, az)
float64 yaw                           # 偏航角
float64 yaw_dot                       # 偏航角速度
float64[3] kx                         # 位置控制增益
float64[3] kv                         # 速度控制增益

uint32 trajectory_id                  # 轨迹ID

# 轨迹状态常量定义
uint8 TRAJECTORY_STATUS_EMPTY = 0
uint8 TRAJECTORY_STATUS_READY = 1  
uint8 TRAJECTORY_STATUS_COMPLETED = 3
uint8 TRAJECTROY_STATUS_ABORT = 4
uint8 TRAJECTORY_STATUS_ILLEGAL_START = 5
uint8 TRAJECTORY_STATUS_ILLEGAL_FINAL = 6
uint8 TRAJECTORY_STATUS_IMPOSSIBLE = 7

# Its ID number will start from 1, allowing you comparing it with 0.
uint8 trajectory_flag
