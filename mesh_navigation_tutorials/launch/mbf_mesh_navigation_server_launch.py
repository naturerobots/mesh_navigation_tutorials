import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            "mesh_map_path",
            description="Path to the mesh file that defines the map.",
        ),
    ]
    mesh_map_path = LaunchConfiguration("mesh_map_path")

    mbf_mesh_nav_config = os.path.join(
        get_package_share_directory("mesh_navigation_tutorials"), "config", "mbf_mesh_nav.yaml"
    )

    mesh_nav_server = Node(
        name="move_base_flex",
        package="mbf_mesh_nav",
        executable="mbf_mesh_nav",
        remappings=[
            ("/move_base_flex/cmd_vel", "/cmd_vel"),
        ],
        parameters=[
            mbf_mesh_nav_config,
            {"mesh_map.mesh_file": mesh_map_path},
        ],
    )

    return LaunchDescription(
        launch_args
        + [
            mesh_nav_server,
        ]
    )
