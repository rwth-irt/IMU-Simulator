# @license BSD-3 https://opensource.org/licenses/BSD-3-Clause
# Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
# Maximilian Nitsch (m.nitsch@irt.rwth-aachen.de)
# All rights reserved.

import os

import yaml
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Create the launch description
    ld = LaunchDescription()

    # Declare the path to the config YAML file
    config_file_path = os.path.join(
        get_package_share_directory("imu_simulator_package"),  # noqa
        "config",  # noqa
        "stim300.yaml",  # noqa
    )

    # Open the YAML file and load the parameters
    with open(config_file_path, "r") as file:
        config = yaml.safe_load(file)

    topic_name_odom_arg = DeclareLaunchArgument(
        "topic_name_odom",
        default_value="/nanoauv/odometry",
        description="Topic name of the ground truth odometry from vehicle",
    )

    topic_name_accel_arg = DeclareLaunchArgument(
        "topic_name_accel",
        default_value="/nanoauv/accel",
        description="Topic name of the ground truth acceleration from vehicle",
    )

    # Add the launch argument to the launch description
    ld.add_action(topic_name_odom_arg)
    ld.add_action(topic_name_accel_arg)

    # Create the node
    imu_simulator_package_node = Node(
        package="imu_simulator_package",
        namespace="/nanoauv/sensor/imu",
        executable="imu_simulator_package_node",
        name="imu_simulator_node",
        output="screen",
        parameters=[
            config,
            {"topic_name_odom": LaunchConfiguration("topic_name_odom")},
            {"topic_name_accel": LaunchConfiguration("topic_name_accel")}
        ]
    )

    ld.add_action(imu_simulator_package_node)

    return ld
