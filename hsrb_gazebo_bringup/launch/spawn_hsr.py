#!/usr/bin/env python3
# Copyright (c) 2024 TOYOTA MOTOR CORPORATION
# All rights reserved.
# Redistribution and use in source and binary forms, with or without
# modification, are permitted (subject to the limitations in the disclaimer
# below) provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of the copyright holder nor the names of its contributors may be used
#   to endorse or promote products derived from this software without specific
#   prior written permission.
# NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
# LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
# GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
# OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE.
# -*- coding: utf-8 -*-
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction
)
from launch.conditions import (
    IfCondition,
    UnlessCondition,
)
from launch.substitutions import (
    LaunchConfiguration,
)
from launch_ros.actions import Node


# (h-yaguchi) In humble, the controller is launched using this method
def create_spawner_node(controller_name, manager_name='/controller_manager'):
    return Node(package='controller_manager',
                executable='spawner',
                arguments=[controller_name, '--controller-manager', manager_name])


def declare_arguments(context, robot_name):
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument('gazebo_visualization', default_value="false",
                                                    description='gazebo visualization status'))
    declared_arguments.append(DeclareLaunchArgument('use_odom_ground_truth', default_value='true',
                                                    description='Use ground truth odometry at omni base controller.'))
    declared_arguments.append(DeclareLaunchArgument('runtime_config_package', default_value='hsrb_gazebo_bringup',
                                                    description='Package with the controller\'s configuration.'))
    declared_arguments.append(DeclareLaunchArgument('controllers_file', default_value='gazebo_ros2_control.yaml',
                                                    description='YAML file with the controllers configuration.'))
    declared_arguments.append(DeclareLaunchArgument('robot_pos', default_value='0.0,0.0,0.0,0.0',
                                                    description='spawn_entity.py robot position.'))
    declared_arguments.append(DeclareLaunchArgument('ground_truth_xyz', default_value='0.0\\ 0.0\\ 0.0',
                                                    description='gazebo ground truth xyz position.'))
    declared_arguments.append(DeclareLaunchArgument('ground_truth_rpy', default_value='0.0\\ 0.0\\ 0.0',
                                                    description='gazebo ground truth rpy position.'))
    return declared_arguments


def create_static_tf_node(parent_frame, child_frame):
    return Node(package='tf2_ros',
                executable='static_transform_publisher',
                name='static_transform_publisher',
                output='log',
                arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0',
                           parent_frame, child_frame])


def generate_image_proc_node(sensor_name: str):
    if 'head' in sensor_name:
        return [Node(package='depth_image_proc',
                     name=sensor_name + '_convert_depth_metric_node',
                     executable='convert_metric_node',
                     remappings=[('image_raw', sensor_name + '/depth_registered/image_rect_raw'),
                                 ('image', sensor_name + '/depth_registered/image_raw'),
                                 ('image/compressed', sensor_name + '/depth_registered/image_raw/compressed'),
                                 ('image/compressedDepth', sensor_name + '/depth_registered/image_raw/compressedDepth'),
                                 ('image/theora', sensor_name + '/depth_registered/image_raw/theora')]),
                Node(package='hsrb_gazebo_bringup',
                     name=sensor_name + '_compressed_rgbd_publisher',
                     executable='compressed_rgbd_publisher',
                     remappings=[('rgb/camera_info', sensor_name + '/rgb/camera_info'),
                                 ('rgb/image', sensor_name + '/image_raw/compressed'),
                                 ('depth/camera_info', sensor_name + '/depth_registered/camera_info'),
                                 ('depth/image', sensor_name + '/depth_registered/image_raw/compressedDepth'),
                                 ('~/image', sensor_name + '/rgbd/compressed')])]
    else:
        return [Node(package='depth_image_proc',
                     name=sensor_name + '_convert_depth_metric_node',
                     executable='convert_metric_node',
                     remappings=[('image_raw', sensor_name + '/depth/image'),
                                 ('image', sensor_name + '/depth/image_raw'),
                                 ('image/compressed', sensor_name + '/depth/image_raw/compressed'),
                                 ('image/compressedDepth', sensor_name + '/depth/image_raw/compressedDepth'),
                                 ('image/theora', sensor_name + '/depth/image_raw/theora')]),
                Node(package='hsrb_gazebo_bringup',
                     name=sensor_name + '_compressed_rgbd_publisher',
                     executable='compressed_rgbd_publisher',
                     remappings=[('rgb/camera_info', sensor_name + '/rgb/camera_info'),
                                 ('rgb/image', sensor_name + '/rgb/image_raw/compressed'),
                                 ('depth/camera_info', sensor_name + '/depth/camera_info'),
                                 ('depth/image', sensor_name + '/depth/image_raw/compressedDepth'),
                                 ('~/image', sensor_name + '/rgbd/compressed')])]


def launch_setup(context,
                 robot_name,
                 robot_pos,
                 ground_truth_xyz,
                 ground_truth_rpy,
                 gazebo_visualization,
                 robot_description):
    robot_pos_list = context.perform_substitution(robot_pos).split(',')
    robot_name_value = context.perform_substitution(robot_name)
    robot_description_value = context.perform_substitution(robot_description)

    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py', output='screen',
        arguments=[
            '-topic',
            'robot_description',
            '-entity',
            robot_name_value,
            '-x',
            robot_pos_list[0],
            '-y',
            robot_pos_list[1],
            '-z',
            robot_pos_list[2],
            '-Y',
            robot_pos_list[3]])

    # These issues are similar to those in hsrb_bringup for the actual machine
    # TODO(Takeshita) /robot_descriptionがあるべきなのでこうしたが，複数ロボット対応等が面倒になる
    joint_state_publisher = Node(package='joint_state_publisher',
                                 executable='joint_state_publisher',
                                 parameters=[{'source_list': ['/joint_states']}, {'use_sim_time': True}],
                                 namespace='whole_body',
                                 remappings=[('robot_description', '/robot_description')])
    robot_state_publisher = Node(package='robot_state_publisher',
                                 executable='robot_state_publisher',
                                 parameters=[{'robot_description': robot_description_value}, {'use_sim_time': True}],
                                 namespace='whole_body',
                                 output={'both': 'log'},
                                 remappings=[('robot_description', '/robot_description')])
    wheel_odom_connector_tf = Node(package='tf2_ros',
                                   executable='static_transform_publisher',
                                   name='static_transform_publisher',
                                   output='log',
                                   arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0',
                                              'base_footprint_wheel', 'base_footprint'])

    laser_odom_node = Node(
        package='ros2_laser_scan_matcher',
        executable='laser_scan_matcher',
        name='laser_scan_matcher',
        output='screen',
        remappings=[('odom', 'laser_odom')],
        parameters=[{'laser_frame': 'base_range_sensor_link'},
                    {'publish_odom': 'laser_odom'}])

    # Using gazebo_ros2_control makes remapping difficult, so relay is used
    # ROS2 support for topic_tools itself is not yet complete
    odom_relay_node_gt = Node(package='hsrb_gazebo_bringup',
                              executable='odom_relay',
                              name='odom_relay',
                              remappings=[('~/input_odom', 'odom_ground_truth'),
                                          ('~/output_odom', 'odom')],
                              condition=IfCondition(LaunchConfiguration('use_odom_ground_truth')))
    odom_to_tf_converter = Node(package='hsrb_gazebo_bringup',
                                executable='odom_to_tf_converter',
                                name='odom_to_tf_converter',
                                parameters=[{'frame_id': 'odom'}],
                                remappings=[('~/input_odom', 'odom_ground_truth')],
                                condition=IfCondition(LaunchConfiguration('use_odom_ground_truth')),
                                output='screen',
                                emulate_tty=True,
                                )
    odom_relay_node_wheel = Node(package='hsrb_gazebo_bringup',
                                 executable='odom_relay',
                                 name='odom_relay',
                                 #  remappings=[('~/input_odom', 'omni_base_controller/wheel_odom'),
                                 #              ('~/output_odom', 'wheel_odom')],
                                 remappings=[('~/input_odom', 'laser_odom'),
                                             ('~/output_odom', 'odom')],
                                 condition=UnlessCondition(LaunchConfiguration('use_odom_ground_truth')))

    # These issues are similar to those in hsrb_bringup for the actual machine
    # TODO(Takeshita) /robot_descriptionがあるべきなのでこうしたが，複数ロボット対応等が面倒になる
    nodes = [laser_odom_node,
             joint_state_publisher,
             robot_state_publisher,
             odom_relay_node_gt,
             odom_to_tf_converter,
             odom_relay_node_wheel,
             wheel_odom_connector_tf,
             create_static_tf_node('head_l_stereo_camera_link', 'head_l_stereo_camera_frame'),
             create_static_tf_node('head_r_stereo_camera_link', 'head_r_stereo_camera_frame'),
             create_static_tf_node('head_rgbd_sensor_link', 'head_rgbd_sensor_rgb_frame'),
             create_spawner_node('joint_state_broadcaster'),
             create_spawner_node('head_trajectory_controller'),
             create_spawner_node('arm_trajectory_controller'),
             create_spawner_node('omni_base_controller'),
             spawn_entity]
    nodes.extend(generate_image_proc_node('head_rgbd_sensor'))
    nodes.extend(generate_image_proc_node('hand_camera'))

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [OpaqueFunction(function=declare_arguments,
                        args=[LaunchConfiguration('robot_name')]),
         OpaqueFunction(function=launch_setup,
                        args=[LaunchConfiguration('robot_name'),
                              LaunchConfiguration('robot_pos'),
                              LaunchConfiguration('ground_truth_xyz'),
                              LaunchConfiguration('ground_truth_rpy'),
                              LaunchConfiguration('gazebo_visualization'),
                              LaunchConfiguration('robot_description')])])
