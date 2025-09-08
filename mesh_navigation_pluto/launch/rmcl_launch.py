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
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, PythonExpression, LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    # path to this pkg
    pkg_mesh_navigation_tutorials = get_package_share_directory("mesh_navigation_pluto")

    # Loading a map files with the following extension
    mesh_nav_map_ext = ".ply"

    available_map_names = [
        f[:-len(mesh_nav_map_ext)]
        for f in os.listdir(os.path.join(pkg_mesh_navigation_tutorials, "maps"))
        if f.endswith(mesh_nav_map_ext)
    ]

    # Launch arguments
    launch_args = [
        DeclareLaunchArgument(
            "map_name",
            description="Name of the map to be used for navigation"
            + '(see mesh_navigation_tutorials\' "maps" directory).',
            default_value=LaunchConfiguration("world_name"),
            choices=available_map_names,
        ),
        DeclareLaunchArgument(
            "localization",
            description="How the robot shall localize itself",
            default_value="ground_truth",
            choices=["ground_truth", "rmcl_micpl"],
        ),
        DeclareLaunchArgument(
            "obstacle_segmentation",
            description="Method to segment LiDAR point for obstacles",
            default_value="none",
            choices=["none", "ground_truth", "rmcl_seg"],
        ),
        DeclareLaunchArgument(
            "start_rviz",
            description="Whether rviz shall be started.",
            default_value="True",
            choices=["True", "False"],
        ),
    ]

    map_name = LaunchConfiguration("map_name")
    localization = LaunchConfiguration("localization")
    obstacle_segmentation = LaunchConfiguration("obstacle_segmentation")


    rmcl_config = PathJoinSubstitution([
                    pkg_mesh_navigation_tutorials, 
                    "config", 
                    "rmcl.yaml"])

    mesh_map_path = PathJoinSubstitution([
                    pkg_mesh_navigation_tutorials,
                    "maps",
                    PythonExpression(['"', map_name, '.ply"']),
                ])

    # conversion
    rmcl_pc2_to_o1dn_conv = Node(
        condition=IfCondition(PythonExpression([
            '"', localization, '" == "rmcl_micpl"', 
            ' or ',
            '"', obstacle_segmentation, '" == "rmcl_seg"'])),
        package="rmcl_ros",
        executable="conv_pc2_to_o1dn_node",
        name="rmcl_lidar3d_conversion",
        output="screen",
        remappings=[
            ("input", "/cloud"),
            ("output", "/rmcl_inputs/cloud"),
        ],
        parameters=[
            rmcl_config,
            {
                "use_sim_time": True
            },
        ],
    )

    # MICP-L (Mesh ICP localization) from RMCL package
    rmcl_micpl = Node(
        condition=IfCondition(PythonExpression(['"', localization, '" == "rmcl_micpl"'])),
        package="rmcl_ros",
        executable="micp_localization_node",
        name="rmcl_micpl",
        output="screen",
        parameters=[
            rmcl_config,
            {
                "use_sim_time": True,
                "map_file": mesh_map_path
            },
        ],
    )

    # MICP-L (Mesh ICP localization) from RMCL package
    rmcl_seg = Node(
        condition=IfCondition(PythonExpression(['"', obstacle_segmentation, '" == "rmcl_seg"'])),
        package="rmcl_ros",
        executable="o1dn_map_segmentation_embree_node",
        name="rmcl_seg",
        output="screen",
        remappings=[
            ("scan", "/rmcl_inputs/cloud"),
            ("outlier_map", "~/outlier_map"),
            ("outlier_scan", "obstacle_points"),
        ],
        parameters=[
            rmcl_config,
            {
                "use_sim_time": True,
                "map_file": mesh_map_path
            },
        ],
    )

    return LaunchDescription(
        launch_args
        + [
            rmcl_pc2_to_o1dn_conv,
            rmcl_micpl,
            rmcl_seg
        ]
    )

