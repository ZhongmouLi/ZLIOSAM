import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def generate_launch_description():

    share_dir = get_package_share_directory('lio_sam')
    parameter_file = LaunchConfiguration('params_file')
    xacro_path = os.path.join(share_dir, 'config', 'robot.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_path).toxml()
    rviz_config_file = os.path.join(share_dir, 'config', 'rviz2.rviz')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            share_dir, 'config', 'params.yaml'),
        description='FPath to the ROS2 parameters file to use.')

    print("urdf_file_name : {}".format(xacro_path))




    return LaunchDescription([
        params_declare,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('ros_gz_sim'),
                '/launch',
                '/gz_sim.launch.py'
            ]),
            launch_arguments={'gz_args': '-r empty.sdf'}.items()
        ),
        Node(
                package='ros_gz_sim', 
                executable='create',
                arguments=['-topic', '/robot_description',
                                   '-z', '0.1'],
                output='screen'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description_raw
            }]
        ),

        Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=  [

                            '/clock'                           + '@rosgraph_msgs/msg/Clock'   + '[' + 'ignition.msgs.Clock',

                            '/lzm/points/points'  + '@sensor_msgs/msg/PointCloud2'   + '[' + 'ignition.msgs.PointCloudPacked',

                            # '/lzm/odometry' + '@nav_msgs/msg/Odometry'     + '[' + 'ignition.msgs.Odometry',

                            '/lzm/scan'     + '@sensor_msgs/msg/LaserScan' + '[' + 'ignition.msgs.LaserScan',

                            '/lzm/tf'       + '@tf2_msgs/msg/TFMessage'    + '[' + 'ignition.msgs.Pose_V',

                            '/lzm/imu'      + '@sensor_msgs/msg/Imu'       + '[' + 'ignition.msgs.IMU',

                            '/joint_state' + '@sensor_msgs/msg/JointState' + '[' + 'ignition.msgs.Model',

                            ],

                parameters= [{'qos_overrides./lzm.subscriber.reliability': 'reliable'}],

                remappings= [

                            # ('/lzm/cmd_vel',  '/cmd_vel'),

                            ('/lzm/points/points', '/veldoyne_points'   ),

                            ('/lzm/scan',     '/scan'   ),

                            ('/lzm/tf',       '/tf'     ),

                            ('/lzm/imu',      '/imu_raw'),

                            ('/joint_state', 'joint_states')

                            ],

                output='screen'
    ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments='0.0 0.0 0.0 0.0 0.0 0.0 map odom'.split(' '),
            parameters=[parameter_file],
            output='screen'
            ),
        Node(
            package='lio_sam',
            executable='lio_sam_imuPreintegration',
            name='lio_sam_imuPreintegration',
            parameters=[parameter_file],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_imageProjection',
            name='lio_sam_imageProjection',
            parameters=[parameter_file],
            output='screen'
        ),    
        Node(
            package='lio_sam',
            executable='lio_sam_featureExtraction',
            name='lio_sam_featureExtraction',
            parameters=[parameter_file],
            output='screen'
        ),
        # problem 
        Node(
            package='lio_sam',
            executable='lio_sam_mapOptimization',
            name='lio_sam_mapOptimization',
            parameters=[parameter_file],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ])
