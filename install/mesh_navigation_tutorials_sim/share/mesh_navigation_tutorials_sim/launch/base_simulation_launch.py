# Copyright 2024 Nature Robots GmbH
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Nature Robots GmbH nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitution import Substitution, Text



def generate_launch_description():
    # path to this pkg
    pkg_mesh_navigation_tutorials_sim = get_package_share_directory(
        "mesh_navigation_tutorials_sim"
    )

    # Launch arguments
    launch_args = [DeclareLaunchArgument(
            "world_pkg",
            description="Package of the world to simulate",
        ),
        DeclareLaunchArgument(
            "world_name",
            description="Name of the world to simulate"
            + '(see mesh_navigation_tutorials\' "worlds" directory).'
        ),
        DeclareLaunchArgument(
            "start_gazebo_gui",
            description="Start Gazebo GUI",
            default_value="True",
            choices=["True", "False"],
        ),
    ]

    world_pkg = FindPackageShare(LaunchConfiguration("world_pkg"))
    world_name = LaunchConfiguration("world_name")
    world_path = PathJoinSubstitution(
        [
            world_pkg,
            "worlds",
            PythonExpression(['"', world_name, '" + ".sdf"']),
        ]
    )
    start_gazebo_gui = LaunchConfiguration("start_gazebo_gui")

    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([pkg_mesh_navigation_tutorials_sim, "urdf/ceres.urdf.xacro"]),
            " name:=robot",
            " prefix:='robot'",
            " is_sim:=true",
        ]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "publish_frequency": 100.0,
                "robot_description": robot_description,
            }
        ],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={
            "gz_args": [
                "-r ",
                world_path,  # which world to load
                PythonExpression(
                    ['"" if ', start_gazebo_gui, ' else " -s"']
                ),  # whether to start gui
            ]
        }.items(),
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_robot",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            # The robot's name in simulation is always "robot", regardless of which model is chosen
            # This facilitates easier topic bridging.
            "robot",
            "-z",
            "0.1",
        ],
        parameters=[
            {"use_sim_time": True},
        ],
    )

    # Bridge between ROS and Gazebo
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": PathJoinSubstitution(
                    [pkg_mesh_navigation_tutorials_sim, "config", "ros_gazebo_bridge.yaml"]
                ),
            }
        ],
        output="screen",
    )

    return LaunchDescription(launch_args + [gz_sim, spawn_robot, bridge, robot_state_publisher])
