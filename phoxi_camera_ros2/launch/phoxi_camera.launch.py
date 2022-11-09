# Copyright 2022 Amadeusz Szymko
# Perception for Physical Interaction Laboratory at Poznan University of Technology
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.events.lifecycle import ChangeState
from launch_ros.events import matches_node_name
from launch_ros.event_handlers import OnStateTransition
from launch.actions import LogInfo
from launch.events import matches_action
from launch.event_handlers.on_shutdown import OnShutdown
import lifecycle_msgs.msg


def generate_launch_description():
    phoxi_camera_launch_pkg_prefix = get_package_share_directory("phoxi_camera_ros2")
    rviz_cfg_path = os.path.join(phoxi_camera_launch_pkg_prefix, "rviz/default.rviz")
    urdf_file_path = os.path.join(phoxi_camera_launch_pkg_prefix, "urdf/M+_Motion_Cam-3D.xacro")

    phoxi_config_param = DeclareLaunchArgument(
        'phoxi_config_param_file',
        default_value=[phoxi_camera_launch_pkg_prefix, '/param/defaults.param.yaml'],
        description='Phoxi node config.'
    )

    sigterm_timeout_param = DeclareLaunchArgument(
        'sigterm_timeout',
        default_value='40',
        description='Timeout after node shutdown in order to close driver properly.'
    )

    with_rviz_param = DeclareLaunchArgument(
        'with_rviz',
        default_value='True',
        description='Launch RVIZ2 in addition to phoxi node.'
    )

    phoxi_camera_node = LifecycleNode(
        package='phoxi_camera_ros2',
        executable='phoxi_camera_node_exe',
        name='phoxi_camera',
        namespace='phoxi',
        output='screen',
        # emulate_tty=True,
        parameters=[
            LaunchConfiguration('phoxi_config_param_file')
        ],
        arguments=['--ros-args', '--log-level', 'info', '--enable-stdout-logs'],
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(phoxi_camera_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=phoxi_camera_node, goal_state='inactive',
            entities=[
                LogInfo(
                    msg="[LifecycleLaunch] PhoXiCamera driver node is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(phoxi_camera_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    shutdown_event = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                EmitEvent(event=ChangeState(
                  lifecycle_node_matcher=matches_node_name(node_name="phoxi_camera"),
                  transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVE_SHUTDOWN,
                )),
                LogInfo(
                    msg="[LifecycleLaunch] PhoXiCamera driver node is exiting."),
            ],
        )
    )

    phoxi_camera_api = ExecuteProcess(
        cmd=["PhoXiControl"],
        additional_env={"PHOXI_WITHOUT_DISPLAY": "ON"},
        shell=True,
        emulate_tty=True,
        on_exit=shutdown_event,
        output='screen',
        log_cmd=True,
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', str(rviz_cfg_path)],
        condition=IfCondition(LaunchConfiguration('with_rviz'))
    )

    state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='phoxi_state_publisher',
        namespace='phoxi',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': Command(['xacro', ' ', urdf_file_path])
        }]
    )

    ld = LaunchDescription([
        phoxi_config_param,
        sigterm_timeout_param,
        with_rviz_param,
        phoxi_camera_node,
        activate_event,
        configure_event,
        shutdown_event,
        phoxi_camera_api,
        rviz2_node,
        state_publisher_node
    ])
    return ld
