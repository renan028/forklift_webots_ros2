#!/usr/bin/env python

"""Launch Webots and the controller."""

import os

import launch
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch_ros.events.lifecycle import ChangeState
from launch_ros.events.lifecycle import matches_node_name
from launch_ros.event_handlers import OnStateTransition
from launch.actions import LogInfo
from launch.events import matches_action
from launch.event_handlers.on_shutdown import OnShutdown
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import lifecycle_msgs.msg

from ament_index_python.packages import get_package_share_directory

package_name = 'forklift_webots_ros2'
package_robot_description = 'forklift_robot_description'

def generate_launch_description():
    robot_dir = get_package_share_directory(package_robot_description)

    # Webots Simulator
    webots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('webots_ros2_core'), 
                'launch', 'webots_launch.py')
        ),
        launch_arguments={
            'world': os.path.join(robot_dir, 'forklift_webots', 'worlds', 
                'warehouse.wbt'),
            'output': 'screen'
        }.items()
    )

    # Webots Node
    webots_node = Node(
        package='webots_ros2_cpp',
        executable='wb_task_node',
        output='screen',
    )

    return launch.LaunchDescription([
        webots,
        webots_node
    ])