# Mesh Navigation Tutorials on Real World Ceres maps

This repository contains the "ceres" environments. A group of newer maps recorded in reality during the benchmarking of the MeshMPPI controller.

## Download GIT LFS

The following maps are available within this package:

- `cic_outdoor`
- `agro_technicum`
- `fh_aachen`

While being in the source directory of this repository, download all Ceres maps by entering

```bash
git lfs pull --include="mesh_navigation_ceres*"
```

or specific ones by calling

```bash
git lfs pull --include="mesh_navigation_ceres*/**/cic_outdoor*"
```

## Install Ceres Maps

Compile your ROS workspace to install the downloaded Ceres maps

```bash
colcon build
```

## Quick-Start

```bash
ros2 launch mesh_navigation_ceres mesh_navigation_ceres_launch.py world_name:=cic_outdoor
```

> [!NOTE]
> Since the maps are quite large the startup might take some time to load.


## Acknowledgements

We thank the [MASKOR](https://maskor.fh-aachen.de/) institute for supplying the TLS data of the `fh_aachen` environment for public use.
