# Mesh Navigation Tutorials on Real World pluto maps

This repository contains the "pluto" environments used in mesh navigation papers published before 2024. If you want to reproduce results from those papers, we recommend using the ROS 1 version instead: [https://github.com/uos/pluto_robot](https://github.com/uos/pluto_robot).

## Download GIT LFS

The following maps are available within this package:

- `physics_campus_uos`
- `botanical_garden_osnabrueck`
- `stone_quarry_brockum`

While being in the source directory of this repository, download all Pluto maps by entering

```bash
git lfs pull --include="mesh_navigation_pluto*"
```

or specific ones by calling

```bash
git lfs pull --include="mesh_navigation_pluto*/**/physics_campus_uos*"
```

## Install Pluto Maps

Compile your ROS workspace to install the downloaded Pluto maps

```bash
colcon build
```

## Quick-Start

```bash
ros2 launch mesh_navigation_pluto mesh_navigation_pluto_launch.py world_name:=physics_campus_uos
```

> [!NOTE]
> Since the maps are quite large the startup might take some time to load.

