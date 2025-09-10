# Copyright 2025 Nature Robots GmbH
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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution


def generate_launch_description():
    # path to this pkg
    pkg_mesh_navigation_ceres_sim = get_package_share_directory(
        "mesh_navigation_ceres_sim"
    )

    pkg_mesh_navigation_tutorials_sim = get_package_share_directory(
        "mesh_navigation_tutorials_sim"
    )

    # Launch arguments
    available_world_names = [
        f[:-4]
        for f in os.listdir(os.path.join(pkg_mesh_navigation_ceres_sim, "worlds"))
        if f.endswith(".sdf")
    ]

    # Launch arguments (override/specialize base arguments)
    launch_args = [
        DeclareLaunchArgument(
            "world_name",
            description="Name of the world to simulate"
            + '(see mesh_navigation_tutorials\' "worlds" directory).',
            default_value=available_world_names[0],
            choices=available_world_names,
        ),
    ]

    meshnav_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_mesh_navigation_tutorials_sim, "launch", "base_simulation_launch.py"]
            )
        ),
        launch_arguments={
            "world_pkg": "mesh_navigation_ceres_sim",
            "world_name": LaunchConfiguration("world_name")
        }.items(),
    )

    return LaunchDescription(launch_args + [meshnav_sim])
