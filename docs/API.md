# surface_acquisition API reference

This page is generated from the Doxygen comments in the source code. To browse the same
documentation as HTML, run `doxygen Doxyfile` in the repository folder and open
`doxygen/html/index.html`.

It covers the C++ classes and functions and the scripts. The ROS interface of the nodes
(actions, services, topics and parameters) is described in the [README](../README.md).

## Contents

- [`include/surface_acquisition/cloud_filter.hpp`](#includesurface_acquisitioncloud_filterhpp)
  - [`surface_acquisition::FilterOptions`](#struct-surface_acquisitionfilteroptions)
  - [`surface_acquisition::FilterStats`](#struct-surface_acquisitionfilterstats)
  - [`surface_acquisition::FilterResult`](#struct-surface_acquisitionfilterresult)
- [`include/surface_acquisition/cloud_filter_node.hpp`](#includesurface_acquisitioncloud_filter_nodehpp)
  - [`surface_acquisition::CloudFilterNode`](#class-surface_acquisitioncloudfilternode)
- [Scripts](#scripts)

## include/surface_acquisition/cloud_filter.hpp

### struct `surface_acquisition::FilterOptions`

Tuning of the filtering pipeline (node parameters).

All lengths in metres, expressed in the sensor frame (z = depth along the optical axis).

#### Public attributes

| Member | Type | Description |
|---|---|---|
| `min_depth` | `double` |  |
| `max_depth` | `double` |  |
| `x_min` | `double` |  |
| `x_max` | `double` |  |
| `y_min` | `double` |  |
| `y_max` | `double` |  |
| `voxel_leaf` | `double` |  |
| `sor_mean_k` | `int` |  |
| `sor_stddev_mul` | `double` |  |
| `cluster_enabled` | `bool` |  |
| `cluster_tolerance` | `double` |  |
| `cluster_min_size` | `int` |  |
| `cluster_max_size` | `int` |  |
| `normal_k_search` | `int` |  |
| `normal_radius` | `double` |  |
| `threads` | `int` | OpenMP threads for the normals, 0 = automatic. |

### struct `surface_acquisition::FilterStats`

Number of points after every stage of the pipeline.

#### Public attributes

| Member | Type | Description |
|---|---|---|
| `input` | `std::size_t` | input points, invalid (NaN) ones included |
| `cropped` | `std::size_t` | after the PassThrough crop (invalid points dropped) |
| `downsampled` | `std::size_t` | after the voxel grid |
| `denoised` | `std::size_t` | after the statistical outlier removal |
| `clusters` | `std::size_t` | clusters found (0 if clustering is disabled) |
| `clustered` | `std::size_t` | points of the selected cluster |
| `output` | `std::size_t` | output points with a valid normal |
| `t_crop` | `double` |  |
| `t_voxel` | `double` |  |
| `t_sor` | `double` |  |
| `t_cluster` | `double` |  |
| `t_normals` | `double` |  |

### struct `surface_acquisition::FilterResult`

Output of filterCloud.

#### Public attributes

| Member | Type | Description |
|---|---|---|
| `cloud` | `CloudN::Ptr` | largest cluster with normals, in the input frame |
| `before_clustering` | `Cloud::Ptr` | cloud after crop + voxel grid + SOR (for tuning) |
| `stats` | `FilterStats` |  |

### Free functions and types

#### `Point`

```cpp
using surface_acquisition::Point = pcl::PointXYZ
```

#### `Cloud`

```cpp
using surface_acquisition::Cloud = pcl::PointCloud<Point>
```

#### `PointN`

```cpp
using surface_acquisition::PointN = pcl::PointNormal
```

#### `CloudN`

```cpp
using surface_acquisition::CloudN = pcl::PointCloud<PointN>
```

#### `validate`

```cpp
std::string validate(const FilterOptions &o)
```

Returns an empty string if the options are consistent, the reason otherwise.

#### `filterCloud`

```cpp
FilterResult filterCloud(const Cloud::ConstPtr &input, const FilterOptions &o)
```

Filtering pipeline of node 1, independent of ROS: PassThrough crop -&gt; voxel grid -&gt; statistical outlier removal -&gt; Euclidean clustering (largest cluster) -&gt; normal estimation.

The crop comes before the voxel grid: it is cheaper (one comparison per point) and it bounds the extent of the cloud, so the voxel grid does not overflow its index range with small leaves.

The normals are computed last, on the selected cluster only, and are oriented towards the sensor (origin of the input frame).

## include/surface_acquisition/cloud_filter_node.hpp

### class `surface_acquisition::CloudFilterNode`

Inherits from `rclcpp::Node`.

Node 1 of the pipeline (live acquisition): continuously filters the camera cloud and publishes the largest cluster with normals, for Rviz and for the acquire service of node 2 (cloud_merger_node).

Parameters can be changed at runtime; invalid combinations are rejected.

#### Public functions

##### `CloudFilterNode`

```cpp
CloudFilterNode(const rclcpp::NodeOptions &options=rclcpp::NodeOptions())
```

Declare the parameters, subscribe to the camera cloud and create the publishers.

**Parameters:**

- `options`: Node options.

## Scripts

### `scripts/fake_realsense.py`

```
Simulated wrist RealSense for testing cloud_filter_node without hardware.

The camera looks down at a table (z = 0) with an object on it. The raw cloud
also contains what each filter stage of node 1 has to remove:

- the gripper fingers right in front of the lens  -> depth crop (min_depth)
- the floor around the table, 75 cm below it      -> depth crop (max_depth)
- isolated flying pixels in the field of view     -> statistical outlier removal
- a cable hanging between camera and table        -> largest cluster
- invalid (NaN) pixels                            -> dropped by the crop

The world -> camera TF is broadcast too, and the camera position can be changed
at runtime (ros2 param set), so the simulator also drives node 2 through node 1.
```
