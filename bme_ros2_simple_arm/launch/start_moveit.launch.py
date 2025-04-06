import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_bme_ros2_simple_arm_moveit_config = get_package_share_directory('bme_ros2_simple_arm_moveit_config')

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bme_ros2_simple_arm_moveit_config, 'launch', 'move_group.launch.py'),
        )
    )

    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bme_ros2_simple_arm_moveit_config, 'launch', 'moveit_rviz.launch.py'),
        )
    )

    set_moveit_simtime_parameter = ExecuteProcess(
        cmd=['ros2', 'param', 'set', '/move_group', 'use_sim_time', 'true'],
        output='screen'
    )

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(moveit_launch)
    launchDescriptionObject.add_action(moveit_rviz_launch)
    launchDescriptionObject.add_action(set_moveit_simtime_parameter)

    return launchDescriptionObject