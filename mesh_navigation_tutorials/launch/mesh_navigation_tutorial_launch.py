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
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, PythonExpression, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    # path to this pkg
    pkg_mesh_navigation_tutorials_sim = get_package_share_directory(
        "mesh_navigation_tutorials_sim"
    )
    pkg_mesh_navigation_tutorials = get_package_share_directory("mesh_navigation_tutorials")

    # Comment Alex: One can have different maps for same worlds
    # Is this to much choice for a tutorial?

    # Launch arguments
    available_map_names = [
        f[:-4]
        for f in os.listdir(os.path.join(pkg_mesh_navigation_tutorials, "maps"))
        if f.endswith(".dae")
    ]

    launch_args = [
        DeclareLaunchArgument(
            "map_name",
            description="Name of the map to be used for navigation"
            + '(see mesh_navigation_tutorials\' "maps" directory).',
            default_value=LaunchConfiguration("world_name"),
            choices=available_map_names,
        ),
        DeclareLaunchArgument(
            "localization_type",
            description="How the robot shall localize itself",
            default_value="ground_truth",
            choices=["ground_truth", "rmcl_micpl"],
        ),
        DeclareLaunchArgument(
            "start_rviz",
            description="Whether rviz shall be started.",
            default_value="True",
            choices=["True", "False"],
        ),
    ]
    map_name = LaunchConfiguration("map_name")
    world_name = LaunchConfiguration("world_name")
    start_rviz = LaunchConfiguration("start_rviz")
    localization_type = LaunchConfiguration("localization_type")

    # suggestion:
    # - every world has one map with same name as default
    # - only if map_name is specified another map than default is loaded
    # map_name = world_name

    # Launch simulation environment and robot
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_mesh_navigation_tutorials_sim, "launch", "simulation_launch.py"]
            )
        )
    )

    # odom -> base_footprint
    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            PathJoinSubstitution([pkg_mesh_navigation_tutorials, "config", "ekf.yaml"]),
        ],
    )

    # Ground truth map localization
    map_loc_gt = Node(
        package="mesh_navigation_tutorials_sim",
        executable="ground_truth_localization_node",
        name="ground_truth_localization_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "gz_parent_frame": world_name,
                "gz_child_frame": "robot",
                "ros_parent_frame": "map",
                "ros_child_frame": "base_footprint",
                "ros_odom_frame": "odom",
            }
        ],
        condition=IfCondition(PythonExpression(['"', localization_type, '" == "ground_truth"'])),
    )

    # RMCL Localization
    map_loc_rmcl_micp = Node(
        package="rmcl",
        executable="micp_localization",
        name="micp_localization",
        output="screen",
        remappings=[
            ("pose_wc", "/initialpose"),
        ],
        parameters=[
            {
                "use_sim_time": True,
                "map_file": PathJoinSubstitution(
                    [
                        pkg_mesh_navigation_tutorials,
                        "maps",
                        PythonExpression(['"', map_name, '" + ".dae"']),
                    ]
                ),
            },
            PathJoinSubstitution([pkg_mesh_navigation_tutorials, "config", "rmcl_micpl.yaml"]),
        ],
        condition=IfCondition(PythonExpression(['"', localization_type, '" == "rmcl_micpl"'])),
    )

    # Move Base Flex
    move_base_flex = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_mesh_navigation_tutorials, "launch", "mbf_mesh_navigation_server_launch.py"]
            )
        ),
        launch_arguments={
            "mesh_map_path": PathJoinSubstitution(
                [
                    pkg_mesh_navigation_tutorials,
                    "maps",
                    PythonExpression(['"', map_name, '" + ".ply"']),
                ]
            ),
            "mesh_map_working_path": PythonExpression(['"', map_name, '" + ".h5"'])
        }.items(),
    )

    # Start rviz, if desired
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        parameters=[
            {"use_sim_time": True},
        ],
        arguments=[
            "-d",
            PathJoinSubstitution([pkg_mesh_navigation_tutorials, "rviz", "default.rviz"]),
        ],
        condition=IfCondition(start_rviz),
    )

    return LaunchDescription(
        launch_args
        + [
            simulation,
            ekf,
            map_loc_gt,
            map_loc_rmcl_micp,
            move_base_flex,
            rviz,
        ]
    )
