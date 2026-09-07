# Modular 3D Localizer

`modular_3d_localizer` is a ROS 2 Humble framework for full SE(3) localization
of an incoming 3D point cloud against a static PCD or PLY map.

## Input and TF contract

Required inputs are a static map, a `sensor_msgs/msg/PointCloud2` scan topic,
an initial `map -> base_frame` pose on `/initialpose`, and a resolvable
`base_frame -> lidar_frame` transform.

`base_frame -> lidar_frame` should normally be published by the user's URDF,
sensor driver, or `robot_state_publisher`. The launch file can publish one
parameterized static transform only when no external TF source exists.

An optional `nav_msgs/msg/Odometry` topic supplies a motion prior. The
localizer does not require a particular frontend: wheel odometry, visual-
inertial odometry, LiDAR-inertial odometry, and LiDAR odometry are all valid
as long as they use the configured `odom_frame -> base_frame` convention.

With `tf_output_mode:=map_to_odom`, the resulting tree is:

```text
map -> odom       published by this package
odom -> base      published by the optional odometry frontend
base -> lidar     published by URDF, sensor driver, or static TF
```

## Profiles and launch overrides

All ROS parameters have conservative defaults in `config/defaults.yaml`.
`config/generic_fast_gicp.yaml` is the baseline FastGICP tracking profile.
The selected `params_file` is layered over defaults; a non-empty launch
argument overrides the profile.

Minimal FastGICP bringup:

```bash
ros2 launch modular_3d_localizer localizer.launch.py \
  map_path:=/data/map.pcd \
  point_cloud_topic:=/lidar/points \
  base_frame:=base_link \
  lidar_frame:=lidar
```

If the external TF tree already contains `base_link -> lidar`, do not enable
`publish_base_lidar_tf`.

## Required FastGICP backend

FastGICP is the required registration dependency and the default backend for
this package. PCL GICP and coarse-to-fine PCL GICP remain available as
comparison or extension backends.

```bash
colcon build --packages-select modular_3d_localizer --symlink-install \
  --cmake-args \
  -DFAST_GICP_PREFIX=/path/to/fast_gicp/install
export LD_LIBRARY_PATH=/path/to/fast_gicp/install/lib:$LD_LIBRARY_PATH
```

Then use the profile and override only environment-specific values:

```bash
ros2 launch modular_3d_localizer localizer.launch.py \
  params_file:=/path/to/generic_fast_gicp.yaml \
  map_path:=/data/map.pcd \
  point_cloud_topic:=/lidar/points \
  base_frame:=base_link \
  lidar_frame:=lidar \
  motion_model:=odometry \
  odometry_topic:=/odometry \
  tf_output_mode:=map_to_odom
```

For a sensor without a TF publisher, add its calibrated extrinsic explicitly:

```bash
publish_base_lidar_tf:=true \
base_lidar_x:=0.0 base_lidar_y:=0.0 base_lidar_z:=0.0 \
base_lidar_roll:=0.0 base_lidar_pitch:=0.0 base_lidar_yaw:=0.0
```

## Backends

| `localization_backend` | Build requirement |
| --- | --- |
| `fast_gicp` | Required baseline dependency |
| `gicp` | Included PCL alternative |
| `coarse_to_fine_gicp` | Included PCL alternative |

To add an algorithm, implement `RegistrationBackend` and register it through
`registration_backend_factory.cpp`; map I/O, scan conversion, TF, motion
prior, validation, and visualization remain unchanged.

## Current limitation: global relocalization

This package currently performs local tracking after an initial pose has been
provided. A rejected registration keeps the last accepted pose and continues
with the next scan; it does not switch to a global relocalization search.
Users must provide a new `/initialpose` when tracking has genuinely lost the
map. A replaceable global relocalization backend is planned, but is not part
of the current baseline.
