import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('peg_insertion')

    use_sim_time = {'use_sim_time': True}
    controllers_yaml = os.path.join(pkg_share, 'config', 'controllers.yaml')

    # 1. Start Gazebo Harmonic with our world file
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-r ' + os.path.join(
                pkg_share, 'worlds', 'assembly_world.sdf')
        }.items(),
    )

    # 2. Bridge Gazebo clock → /clock so ros2_control gets sim time
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # 3. Compile the URDF using Xacro
    robot_description_content = Command([
        'xacro ',
        os.path.join(pkg_share, 'urdf', 'custom_robot.xacro')
    ])

    # 4. Publish the robot description to /robot_description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(robot_description_content, value_type=str)},
                    use_sim_time],
        output='screen',
    )

    # 5. Spawn the robot into Gazebo (delayed so Gazebo is ready)
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_ur5',
                arguments=[
                    '-name', 'ur5_with_hand',
                    '-topic', '/robot_description',
                    '-x', '0.0', '-y', '-0.55', '-z', '1.0',
                ],
                output='screen',
            )
        ],
    )

    # 6. Spawn ros2_control controllers (after robot is loaded)
    #    a) Joint state broadcaster
    spawn_jsb = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster',
                           '--param-file', controllers_yaml],
                parameters=[use_sim_time],
                output='screen',
            )
        ],
    )

    #    b) Arm joint trajectory controller
    spawn_jtc = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_trajectory_controller',
                           '--param-file', controllers_yaml],
                parameters=[use_sim_time],
                output='screen',
            )
        ],
    )

    #    c) Hand joint trajectory controller  ← THIS WAS MISSING
    spawn_hand = TimerAction(
        period=9.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['hand_controller',
                           '--param-file', controllers_yaml],
                parameters=[use_sim_time],
                output='screen',
            )
        ],
    )

    # 7. Launch impedance insertion demo node (delayed until ALL controllers up)
    impedance_node = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='peg_insertion',
                executable='impedance_controller',
                parameters=[use_sim_time],
                output='screen',
            )
        ],
    )

    return LaunchDescription([
        gz_sim,
        clock_bridge,
        robot_state_publisher,
        spawn_robot,
        spawn_jsb,
        spawn_jtc,
        spawn_hand,
        impedance_node,
    ])