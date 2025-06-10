import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PythonExpression
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    """
    适配IR100 SLAM系统的EGO-Planner启动文件
    主要修改：替换仿真话题为真实SLAM系统话题
    """
    
    # 定义参数的 LaunchConfiguration
    obj_num = LaunchConfiguration('obj_num', default=10)
    drone_id = LaunchConfiguration('drone_id', default=0)
    
    map_size_x = LaunchConfiguration('map_size_x', default=50.0)
    map_size_y = LaunchConfiguration('map_size_y', default=25.0)
    map_size_z = LaunchConfiguration('map_size_z', default=2.0)
    
    # 🔧 SLAM适配修改1: 使用IR100 SLAM系统的里程计话题
    # 可选择FastLIO2前端或Localizer定位输出
    use_localization = LaunchConfiguration('use_localization', default=False)
    odom_topic = LaunchConfiguration('odom_topic', default='/fastlio2/lio_odom')  # 默认使用FastLIO2
    
    # 🔧 新增修改: 添加仿真时间参数支持
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    
    # 声明全局参数
    obj_num_cmd = DeclareLaunchArgument('obj_num', default_value=obj_num, description='Number of objects')
    drone_id_cmd = DeclareLaunchArgument('drone_id', default_value=drone_id, description='Drone ID')
    
    map_size_x_cmd = DeclareLaunchArgument('map_size_x', default_value=map_size_x, description='Map size along x')
    map_size_y_cmd = DeclareLaunchArgument('map_size_y', default_value=map_size_y, description='Map size along y')
    map_size_z_cmd = DeclareLaunchArgument('map_size_z', default_value=map_size_z, description='Map size along z')
    
    # 🔧 SLAM适配修改2: 支持切换不同的里程计源
    odom_topic_cmd = DeclareLaunchArgument('odom_topic', default_value=odom_topic, 
                                          description='Odometry topic: /fastlio2/lio_odom or /localization/odom')
    use_localization_cmd = DeclareLaunchArgument('use_localization', default_value=use_localization,
                                                description='Use localization odom instead of FastLIO2')

    # 🔧 SLAM适配修改3: 禁用仿真地图生成器，使用真实SLAM地图
    use_simulation_map = LaunchConfiguration('use_simulation_map', default=False)
    use_simulation_map_cmd = DeclareLaunchArgument('use_simulation_map', default_value=use_simulation_map,
                                                  description='Use simulation map generator (false for real SLAM)')
    
    # 🔧 新增修改: 仿真时间参数声明
    use_sim_time_cmd = DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                                            description='Use simulation time (true for simulation, false for real robot)')
    
    # 🔧 SLAM适配修改4: 地图生成器节点 - 仅在仿真模式下启用
    map_generator_node = Node(
        package='map_generator',
        executable='random_forest',
        name='random_forest',
        output='screen',
        parameters=[
            {'map/x_size': 26.0},
            {'map/y_size': 20.0},
            {'map/z_size': 3.0},
            {'map/resolution': 0.1},
            {'ObstacleShape/seed': 1.0},
            {'map/obs_num': 250},
            {'ObstacleShape/lower_rad': 0.5},
            {'ObstacleShape/upper_rad': 0.7},
            {'ObstacleShape/lower_hei': 0.0},
            {'ObstacleShape/upper_hei': 3.0},
            {'map/circle_num': 250},
            {'ObstacleShape/radius_l': 0.7},
            {'ObstacleShape/radius_h': 0.5},
            {'ObstacleShape/z_l': 0.7},
            {'ObstacleShape/z_h': 0.8},
            {'ObstacleShape/theta': 0.5},
            {'pub_rate': 1.0},
            {'min_distance': 0.8},
            # 🔧 新增修改: 添加仿真时间参数
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(use_simulation_map)  # 只在仿真模式下启用
    )

    # 🔧 SLAM适配修改5: 禁用MockaMap，使用真实地图
    # mockamap_node 完全移除，因为我们使用真实SLAM地图
    
    # 🔧 SLAM适配修改6: 高级参数配置 - 替换话题映射
    advanced_param_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ego_planner'), 'launch', 'advanced_param.launch.py')),
        launch_arguments={
            'drone_id': drone_id,
            'map_size_x_': map_size_x,
            'map_size_y_': map_size_y,
            'map_size_z_': map_size_z,
            'odometry_topic': odom_topic,
            'obj_num_set': obj_num,
            # 🔧 新增修改: 传递仿真时间参数
            'use_sim_time': use_sim_time,
            
            # 🔧 SLAM适配修改7: 替换传感器话题为真实SLAM话题
            'camera_pose_topic': odom_topic,           # 使用里程计作为位姿源
            'depth_topic': '/camera/depth/image_rect_raw', # 使用深度图像作为深度数据
            'cloud_topic': '/fastlio2/body_cloud',    # 使用FastLIO2实时点云
            
            # 相机内参 (如果使用视觉感知需要根据实际相机调整)
            'cx': str(321.04638671875),
            'cy': str(243.44969177246094),
            'fx': str(387.229248046875),
            'fy': str(387.229248046875),
            
            # 规划参数
            'max_vel': str(2.0),
            'max_acc': str(6.0),
            'planning_horizon': str(7.5),
            'use_distinctive_trajs': 'True',
            'flight_type': str(2),
            'point_num': str(4),
            
            # 路径点配置
            'point0_x': str(15.0),
            'point0_y': str(0.0),
            'point0_z': str(1.0),
            
            'point1_x': str(-15.0),
            'point1_y': str(0.0),
            'point1_z': str(1.0),
            
            'point2_x': str(15.0),
            'point2_y': str(0.0),
            'point2_z': str(1.0),
            
            'point3_x': str(-15.0),
            'point3_y': str(0.0),
            'point3_z': str(1.0),
            
            'point4_x': str(15.0),
            'point4_y': str(0.0),
            'point4_z': str(1.0),
        }.items()
    )
    
    # 轨迹服务器节点
    traj_server_node = Node(
        package='ego_planner',
        executable='traj_server',
        name=['drone_', drone_id, '_traj_server'],
        output='screen',
        # 🔧 新增修改: 添加仿真时间参数
        parameters=[
            {'traj_server/time_forward': 1.0},
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('position_cmd', ['drone_', drone_id, '_planning/pos_cmd']),
            ('planning/bspline', ['drone_', drone_id, '_planning/bspline'])
        ]
    )
    
    # 🔧 SLAM适配修改8: 仿真器配置 - 可选择性启用
    use_dynamic_simulation = LaunchConfiguration('use_dynamic_simulation', default=False)
    use_dynamic_simulation_cmd = DeclareLaunchArgument('use_dynamic_simulation', default_value=use_dynamic_simulation,
                                                      description='Use dynamic simulation (false for real robot)')
    
    simulator_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ego_planner'), 'launch', 'simulator.launch.py')),
        launch_arguments={
            'use_dynamic': use_dynamic_simulation,
            'drone_id': drone_id,
            'map_size_x_': map_size_x,
            'map_size_y_': map_size_y,
            'map_size_z_': map_size_z,
            'init_x_': str(-15.0),
            'init_y_': str(0.0),
            'init_z_': str(0.1),
            'odometry_topic': odom_topic,
            # 🔧 新增修改: 传递仿真时间参数
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(use_dynamic_simulation)  # 只在仿真模式下启用
    )
    
    # 构建Launch Description
    ld = LaunchDescription()
    
    # 添加参数声明
    ld.add_action(map_size_x_cmd)
    ld.add_action(map_size_y_cmd)
    ld.add_action(map_size_z_cmd)
    ld.add_action(odom_topic_cmd)
    ld.add_action(obj_num_cmd)
    ld.add_action(drone_id_cmd)
    ld.add_action(use_localization_cmd)
    ld.add_action(use_simulation_map_cmd)
    ld.add_action(use_dynamic_simulation_cmd)
    # 🔧 新增修改: 添加仿真时间参数声明
    ld.add_action(use_sim_time_cmd)

    # 🔧 SLAM适配修改11: 条件性添加节点
    # 仿真相关节点
    ld.add_action(map_generator_node)          # 仅仿真模式
    ld.add_action(simulator_include)           # 仅仿真模式
    
    # 核心规划节点
    ld.add_action(advanced_param_include)      # 始终需要
    ld.add_action(traj_server_node)           # 始终需要
    

    return ld 