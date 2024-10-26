import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution


def generate_launch_description():
    # path to this pkg
    pkg_mesh_navigation_tutorials_sim = get_package_share_directory(
        "mesh_navigation_tutorials_sim"
    )

    # Launch arguments
    available_world_names = [
        f[:-4]
        for f in os.listdir(os.path.join(pkg_mesh_navigation_tutorials_sim, "worlds"))
        if f.endswith(".sdf")
    ]

    # Launch arguments
    launch_args = [
        DeclareLaunchArgument(
            "world_name",
            description="Name of the world to simulate"
            + '(see mesh_navigation_tutorials\' "worlds" directory).',
            default_value=available_world_names[0],
            choices=available_world_names,
        ),
        DeclareLaunchArgument(
            "start_gazebo_gui",
            description="Start Gazebo GUI",
            default_value="True",
            choices=["True", "False"],
        ),
    ]
    world_name = LaunchConfiguration("world_name")
    world_path = PathJoinSubstitution(
        [
            pkg_mesh_navigation_tutorials_sim,
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
            "1",
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
