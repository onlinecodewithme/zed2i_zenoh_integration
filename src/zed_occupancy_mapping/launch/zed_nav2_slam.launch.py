import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    zed_wrapper_dir = get_package_share_directory('zed_wrapper')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory('zed_occupancy_mapping'),
                                   'config', 'slam_params.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')
    
    declare_nav2_params_file_cmd = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(get_package_share_directory('zed_occupancy_mapping'),
                                   'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for Nav2')
    
    # ZED Camera Launch with proper positional tracking
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([zed_wrapper_dir, '/launch/zed_camera.launch.py']),
        launch_arguments={
            'camera_model': 'zed2i',
            'camera_name': 'zed2i',
            'publish_tf': 'true',
            'publish_map_tf': 'false',  # SLAM will handle map->odom transform
            'publish_odom_tf': 'true',  # Enable ZED odometry TF publishing
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'pos_tracking_enabled': 'true',
            'publish_urdf': 'true',
        }.items()
    )
    
    # Point cloud to laser scan converter (needed for SLAM Toolbox)
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/zed2i/zed_node/point_cloud/cloud_registered'),
            ('scan', '/scan')
        ],
        parameters=[{
            'target_frame': 'zed2i_left_camera_frame',
            'transform_tolerance': 0.01,
            'min_height': -0.5,
            'max_height': 2.0,
            'angle_min': -1.5708,  # -90 degrees
            'angle_max': 1.5708,   # 90 degrees
            'angle_increment': 0.0087,  # 0.5 degrees
            'scan_time': 0.3333,
            'range_min': 0.45,
            'range_max': 8.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }]
    )
    
    # SLAM Toolbox Launch
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([slam_toolbox_dir, '/launch/online_async_launch.py']),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    # Nav2 Launch (without AMCL and map_server since SLAM provides map)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_dir, '/launch/navigation_launch.py']),
        launch_arguments={
            'params_file': nav2_params_file,
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    # Robot State Publisher (basic robot description)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': '''
            <?xml version="1.0"?>
            <robot name="zed_robot">
              <link name="base_link">
                <visual>
                  <geometry>
                    <box size="0.3 0.3 0.1"/>
                  </geometry>
                  <material name="blue">
                    <color rgba="0 0 1 1"/>
                  </material>
                </visual>
                <collision>
                  <geometry>
                    <box size="0.3 0.3 0.1"/>
                  </geometry>
                </collision>
              </link>
            </robot>
            '''
        }]
    )
    
    # Static transform from base_link to camera
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'zed2i_base_link']
    )
    
    # Backup odometry publisher (in case ZED odometry fails to start)
    # This provides a minimal odom->base_link transform until ZED takes over
    backup_odom_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='backup_odom_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )
    
    
    
    # RViz2 Launch
    rviz_config_file = os.path.join(get_package_share_directory('zed_occupancy_mapping'), 'config', 'nav2_slam.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_slam_params_file_cmd,
        declare_nav2_params_file_cmd,
        
        zed_launch,
        pointcloud_to_laserscan,
        slam_launch,
        nav2_launch,
        robot_state_publisher,
        static_transform_publisher,
        backup_odom_publisher,
        rviz_node,
    ])
