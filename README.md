# surface_acquisition

`cloud_filter_node` is node 1 of the surface reconstruction pipeline (live acquisition). It subscribes to the point cloud of the wrist RealSense and continuously publishes a filtered cloud: cropped, downsampled, without outliers, reduced to the largest cluster and with normals oriented towards the camera. The cloud is used for display in Rviz and is the input of the `acquire` service of node 2 ([`surface_reconstruction`](https://github.com/JRL-CARI-CNR-UNIBS/surface_reconstruction)).

For a step-by-step guide, which also works without a robot or a camera, see the **[tutorial](docs/TUTORIAL.md)**.

## Requirements

- Ubuntu 24.04, ROS 2 Jazzy
- PCL 1.14 (`libpcl-dev`), `ros-jazzy-pcl-conversions`
- For the real camera: `ros-jazzy-realsense2-camera`

```bash
cd [ROS2_WORKSPACE]/src
git clone https://github.com/JRL-CARI-CNR-UNIBS/surface_acquisition.git
cd ..
rosdep install --from-paths src/surface_acquisition --ignore-src -y
colcon build --packages-select surface_acquisition --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Build in `Release`: PCL is much slower in `Debug`.

## Running

```bash
ros2 launch surface_acquisition cloud_filter.launch.py \
    input_cloud:=/<realsense_points_topic> \
    params_file:=/path/to/my_config.yaml   # optional
```

| Launch argument | Default | Description |
|---|---|---|
| `input_cloud` | `/wrist/depth/color/points` | cloud published by the RealSense driver (`/camera/camera/depth/color/points` with the driver defaults) |
| `output_cloud` | `filtered_cloud` | filtered cloud; it is the default input of node 2, so the two nodes connect without remapping |
| `params_file` | `config/cloud_filter.yaml` | parameter file |

## Pipeline

```
RealSense cloud (camera optical frame: z = depth, x = right, y = down)
 → 1. crop (PassThrough)         z in [min_depth, max_depth], optional x / y limits; drops NaN points
 → 2. voxel grid                 voxel_leaf
 → 3. statistical outlier removal sor_mean_k, sor_stddev_mul
 → 4. Euclidean clustering       only the cluster with the most points is kept
 → 5. normal estimation          on the final cluster, oriented towards the camera
 → publish filtered_cloud (x, y, z, normal_x, normal_y, normal_z, curvature)
```

Implementation choices:

- **Crop before the voxel grid.** The pipeline document puts the voxel grid first; here the order is swapped. The crop costs one comparison per point and removes most of the background, so the voxel grid has less work to do. It also bounds the extent of the cloud: PCL's voxel grid refuses to run (and returns the input unchanged) when the number of voxels overflows its index, which can happen with small leaves and a far background. The result is the same apart from the voxels cut in half by the limits.
- **Normals last, on the final cluster only.** Clustering only uses Euclidean distances, so there is no reason to compute normals for points that are going to be discarded. The normals are oriented towards the camera (origin of the input frame); after the transformation into the world frame, node 2 then gets consistently oriented normals from every acquisition, as Poisson requires.
- **The table stays in the cloud.** The object lies on the table, so table and object are one Euclidean cluster. Clustering removes what is *not* connected to it (cables, the gripper, other objects, isolated flying pixels). The table is cut by node 2 after merging, where it is a plane at constant Z (see the rationale in the pipeline document).
- **Parallel SOR and clustering.** PCL's `StatisticalOutlierRemoval` and `EuclideanClusterExtraction` are single-threaded and took more than 80% of the time per frame. They are reimplemented with the same criteria and the neighbour searches in parallel (OpenMP); the results are identical to PCL's and the pipeline is about 3.5 times faster (≈ 160 ms instead of ≈ 590 ms for a 220k-point frame on 12 threads, with a 3 mm voxel).
- **Header preserved.** The output has the frame *and the timestamp* of the camera frame it comes from, so node 2 looks up the camera pose at the instant the frame was captured, not when it was processed.
- **Empty output is published.** If nothing survives the filters, an empty cloud is published, so that Rviz and node 2 never keep a stale result (node 2 rejects it in `acquire` because of `min_points`).
- **Latest frame only.** The subscription keeps only the last message (`keep_last(1)`, best effort, compatible with any publisher) and `max_rate` limits the processing rate: frames arriving while a cloud is being processed are dropped instead of queued, so the latency does not grow.

## Interfaces

### Topics

| Topic | Type | Dir. | Description |
|---|---|---|---|
| `input_cloud` | `sensor_msgs/PointCloud2` | sub | camera cloud; only the `x`, `y`, `z` fields are used |
| `filtered_cloud` | `sensor_msgs/PointCloud2` | pub | largest cluster with normals, in the camera frame, with the camera timestamp |
| `~/before_clustering` | `sensor_msgs/PointCloud2` | pub | cloud after crop + voxel grid + SOR, only if `publish_debug` is true; useful to tune the clustering |

No services and no TF: the camera pose is used only by node 2.

## Parameters

All parameters can be changed at runtime with `ros2 param set`; they apply from the next frame. A change that makes the configuration inconsistent (e.g. `min_depth ≥ max_depth`, `cluster_tolerance ≤ voxel_leaf`) is rejected with a message. Double parameters need the decimal point (`0.0`, not `0`).

| Parameter | Default | Description |
|---|---|---|
| `min_depth` | `0.15` | [m] minimum depth; removes the gripper and anything right in front of the camera |
| `max_depth` | `1.0` | [m] maximum depth; removes floor, walls, background |
| `x_min` / `x_max` | `0.0` / `0.0` | [m] lateral limits in the camera frame (x = right); disabled when `min >= max` |
| `y_min` / `y_max` | `0.0` / `0.0` | [m] vertical limits in the camera frame (y = down); disabled when `min >= max` |
| `voxel_leaf` | `0.003` | [m] voxel size; ≤ 0 disables |
| `sor_mean_k` | `30` | neighbours used by the statistical outlier removal; ≤ 0 disables |
| `sor_stddev_mul` | `1.0` | points whose mean neighbour distance exceeds mean + `sor_stddev_mul` · σ are removed |
| `cluster_enabled` | `true` | keep only the largest cluster |
| `cluster_tolerance` | `0.01` | [m] maximum distance between points of the same cluster; must be larger than `voxel_leaf` (≈ 3 ×) |
| `cluster_min_size` / `cluster_max_size` | `100` / `10000000` | clusters with a number of points outside these limits are ignored |
| `normal_k_search` | `20` | neighbours for the normals (used when `normal_radius ≤ 0`) |
| `normal_radius` | `0.0` | [m] radius for the normals; if > 0 it replaces `normal_k_search` |
| `threads` | `0` | threads for SOR, clustering and normals; 0 = all cores |
| `max_rate` | `10.0` | [Hz] maximum processing rate; ≤ 0 processes every frame |
| `publish_debug` | `false` | publish `~/before_clustering` |

Every 5 s the node logs the actual rate, the number of points after each stage and the time spent in each stage:

```
5.5 Hz | points: 157579 input -> 111819 crop -> 65455 voxel -> 65157 SOR -> 64028 (largest of 2 clusters)
       -> 64028 with normals | 143 ms (crop 2, voxel 7, SOR 37, cluster 61, normals 34)
```

## Package layout

```
surface_acquisition/
├── CMakeLists.txt, package.xml
├── config/cloud_filter.yaml               default parameters
├── launch/cloud_filter.launch.py
├── include/surface_acquisition/
│   ├── cloud_filter.hpp                   filtering pipeline, independent of ROS
│   └── cloud_filter_node.hpp
├── src/cloud_filter.cpp                   crop, voxel, SOR, clustering, normals
├── src/cloud_filter_node.cpp              ROS node: parameters, topics, logging
├── src/cloud_filter_main.cpp
├── scripts/fake_realsense.py              simulated wrist camera for tests/tutorial
├── test/test_cloud_filter.cpp             unit tests of the pipeline (gtest)
└── docs/TUTORIAL.md
```

The pipeline is in the `cloud_filter_core` library, which does not depend on ROS and can be reused, e.g. to filter `.pcd` files offline. Tests: `colcon test --packages-select surface_acquisition`.

## Authors

Developed by **CARI-JRL**.

Developer and maintainer: Manuel Beschi (<manuel.beschi@unibs.it>), Elisa Zappa

## License

Apache License 2.0, see [LICENSE](LICENSE).

Copyright 2026 CARI-JRL

## API documentation

The API reference of the classes, functions and scripts is in [docs/API.md](docs/API.md) (generated from the
Doxygen comments).

The code is documented with Doxygen. From the package folder run `doxygen Doxyfile` and open
`doxygen/html/index.html`.
