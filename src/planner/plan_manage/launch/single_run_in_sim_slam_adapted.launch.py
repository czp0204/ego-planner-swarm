import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
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
    主要修改：替换仿真话题为真实SLAM系统话题，集成点云融合节点
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

    
    # 🔧 新增修改: 仿真时间参数声明
    use_sim_time_cmd = DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                                            description='Use simulation time (true for simulation, false for real robot)')
    
    # 🔧 新增: 点云融合参数
    enable_cloud_fusion_cmd = DeclareLaunchArgument('enable_cloud_fusion', default_value='True',
                                                   description='Enable cloud fusion node')

    # 🔧 SLAM适配修改5: 禁用MockaMap，使用真实地图
    # mockamap_node 完全移除，因为我们使用真实SLAM地图
    
    # 🔧 新增: 点云融合节点
    cloud_fusion_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('cloud_fusion'), 'launch', 'cloud_fusion.launch.py')),
        launch_arguments={
            'fusion_rate_hz': '10.0',
            'voxel_filter_size': '0.1',
            'map_frame': 'map',
            'body_frame': 'agv_base_link',
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_cloud_fusion'))
    )
    
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
            'camera_pose_topic': '/camera_pose_in_odom',    # 使用专门的相机位姿话题
            'depth_topic': '/camera/camera/depth/image_rect_raw', # 使用正确的深度图像话题
            'cloud_topic': '/fused_cloud',    # 🔧 修改：使用点云融合节点输出的融合点云数据
            
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
    
    # 🔧 新增修改: 相机位姿发布节点 - 解决坐标系不匹配问题
    camera_pose_publisher_node = ExecuteProcess(
        cmd=['python3', '/home/zhz/ir100slam/camera_pose_publisher.py'],
        name='camera_pose_publisher',
        output='screen'
    )
    
    # 🔧 SLAM适配修改8: 仿真器配置 - 可选择性启用
    use_dynamic_simulation = LaunchConfiguration('use_dynamic_simulation', default=False)
    use_dynamic_simulation_cmd = DeclareLaunchArgument('use_dynamic_simulation', default_value=use_dynamic_simulation,
                                                      description='Use dynamic simulation (false for real robot)')
    
    
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
    ld.add_action(use_dynamic_simulation_cmd)
    ld.add_action(enable_cloud_fusion_cmd)  # 🔧 新增
    # 🔧 新增修改: 添加仿真时间参数声明
    ld.add_action(use_sim_time_cmd)

    # 🔧 新增: 点云融合节点
    ld.add_action(cloud_fusion_include)
    
    # 🔧 SLAM适配修改11: 条件性添加节点
    # 仿真相关节点
    
    # 核心规划节点
    ld.add_action(advanced_param_include)      # 始终需要
    ld.add_action(traj_server_node)           # 始终需要
    ld.add_action(camera_pose_publisher_node)  # 始终需要
    

    return ld 