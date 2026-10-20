# Tutorial — live point cloud filtering with `cloud_filter_node`

The tutorial has four parts:

1. **[Simulation test](#1-simulation-test)**: no hardware needed; a simulated wrist camera produces a cloud with the typical defects of a real one, and you can see what each filter removes.
2. **[Chaining with node 2](#2-chaining-with-node-2)**: from the simulated camera to a reconstructed mesh.
3. **[Use with the RealSense](#3-use-with-the-realsense)**: what to change on the real system.
4. **[Parameter tuning and common problems](#4-parameter-tuning-and-common-problems)**

Time required for parts 1 and 2: about 15 minutes.

---

## 0. Build

```bash
cd ~/projects/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/surface_acquisition --ignore-src -y
colcon build --packages-select surface_acquisition --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Remember to run `source ~/projects/ros2_ws/install/setup.bash` in every new terminal.

---

## 1. Simulation test

### 1.1 The simulated camera

`fake_realsense.py` simulates the wrist camera looking down at a table (`z = 0`) with an object on it, from 50 cm. Besides the table and the object, the cloud contains what node 1 has to get rid of:

| In the simulated cloud | Removed by |
|---|---|
| the gripper fingers, 6 cm in front of the lens | crop, `min_depth` |
| the floor around the table, 1.25 m from the camera | crop, `max_depth` |
| invalid pixels (NaN), 2% of the cloud | crop (always) |
| 400 flying pixels scattered in the field of view | statistical outlier removal (and clustering) |
| a cable hanging 15–20 cm above the table | clustering (largest cluster only) |
| depth noise, growing with the square of the distance | voxel grid (averaging) |

It publishes the cloud on `/wrist/depth/color/points`, in the `wrist_depth_optical_frame` frame, and the TF `world → wrist_depth_optical_frame`.

### 1.2 Start-up

**Terminal 1**, the simulated camera:

```bash
ros2 run surface_acquisition fake_realsense.py
```

**Terminal 2**, node 1:

```bash
ros2 launch surface_acquisition cloud_filter.launch.py
```

You should see:

```
[cloud_filter_node]: cloud_filter_node ready: '/wrist/depth/color/points' -> '/filtered_cloud'
```

and, every 5 seconds, a summary like this one:

```
[cloud_filter_node]: 5.5 Hz | points: 157579 input -> 111819 crop -> 65455 voxel -> 65157 SOR
  -> 64028 (largest of 2 clusters) -> 64028 with normals | 143 ms (crop 2, voxel 7, SOR 37, cluster 61, normals 34)
```

Read it from left to right: 157579 points from the camera; 111819 left after the crop (NaN, fingers and floor removed); 65455 after the 3 mm voxel grid; 65157 after the outlier removal; 2 clusters found (table + object, and the cable), the largest one has 64028 points, all with a valid normal. Then the processing time, in total and per stage.

**Terminal 3**, Rviz:

```bash
rviz2
```

In Rviz:

1. *Global Options → Fixed Frame* = `world`
2. *Add → By topic*:
   - `/wrist/depth/color/points` → PointCloud2: the raw camera cloud. Set *Color* to grey and *Size* to 0.002.
   - `/filtered_cloud` → PointCloud2: the output of node 1. Set *Color Transformer* = `AxisColor` to see it in colour over the grey one.
3. Optional: *Add → TF* to see the camera pose.

From above, the raw cloud also shows the floor around the table and the cable hanging above it. The coloured cloud has only the table and the object.

To see the normals, set on `/filtered_cloud` *Color Transformer* = `Intensity` and *Channel Name* = `normal_z`: on the table the normals point up, towards the camera (`normal_z ≈ -1` in the camera frame, since the optical z axis points down), and the colour changes on the sides of the object.

### 1.3 What each filter does

All parameters can be changed while the node is running. Watch the log summary and Rviz after each change.

**Debug output.** To see the cloud just before clustering, enable the debug topic and add `/cloud_filter_node/before_clustering` in Rviz:

```bash
ros2 param set /cloud_filter_node publish_debug true
```

**Crop on the depth.** Move the far limit beyond the floor:

```bash
ros2 param set /cloud_filter_node max_depth 2.0
```

```
2.8 Hz | points: 157579 input -> 153947 crop -> 107691 voxel -> 105909 SOR -> 64129 (largest of 69 clusters) ...
```

`before_clustering` now contains the floor: 40000 more points to process, and 69 clusters instead of 2. The output does not change, because the table is still the largest cluster. That is luck, however. With a less noisy floor, or with the camera higher up, the floor would be the largest cluster and would replace the table in the output. The crop is cheap and makes the result robust: set `max_depth` a little beyond the farthest point of interest.

```bash
ros2 param set /cloud_filter_node max_depth 1.0
```

**Clustering.** Disable it:

```bash
ros2 param set /cloud_filter_node cluster_enabled false
```

```
... -> 65114 (clustering disabled) -> 65114 with normals ...
```

The cable appears in `/filtered_cloud`. Node 2 would merge it into the scan and Poisson would reconstruct it as a piece of surface hanging above the object.

**Near crop.** With clustering still disabled:

```bash
ros2 param set /cloud_filter_node min_depth 0.0
```

The gripper fingers appear right next to the camera origin (zoom in on the TF of the camera). On the real robot, set `min_depth` to just beyond the tool tip.

**Outlier removal.**

```bash
ros2 param set /cloud_filter_node min_depth 0.15
ros2 param set /cloud_filter_node sor_mean_k 0
```

The flying pixels appear in the output as isolated points between the camera and the table. Clustering would remove them too, because they are not connected to the table. The SOR matters for the noise that clustering *cannot* remove, because it is attached to the surface: on the real camera, the "fringes" of flying pixels along the depth discontinuities at the edges of the object.

Restore the defaults:

```bash
ros2 param set /cloud_filter_node sor_mean_k 30
ros2 param set /cloud_filter_node cluster_enabled true
```

**Voxel grid.** A larger leaf means fewer points and faster processing, but less detail:

```bash
ros2 param set /cloud_filter_node voxel_leaf 0.006
```

```
7.3 Hz | points: 157579 input -> 111871 crop -> 16387 voxel -> 16101 SOR -> 15775 (largest of 2 clusters) ... | 74 ms
```

Now try a value that is too large:

```bash
ros2 param set /cloud_filter_node voxel_leaf 0.01
```

```
Setting parameter failed: cluster_tolerance must be larger than voxel_leaf, otherwise neighbouring voxels are split into different clusters
```

The change is rejected and the node keeps working with the previous values. To use a 1 cm leaf, first increase `cluster_tolerance` (e.g. to `0.03`). Note also that double parameters need the decimal point: `voxel_leaf 0` is rejected as an integer, write `0.0`.

```bash
ros2 param set /cloud_filter_node voxel_leaf 0.003
```

When you are happy with the values, save them with `ros2 param dump /cloud_filter_node > my_filter.yaml` and reuse them with `params_file:=my_filter.yaml`.

---

## 2. Chaining with node 2

Node 1 publishes `filtered_cloud`, which is also the default input of `cloud_merger_node`, so the two nodes connect without remapping. `fake_realsense.py` also broadcasts the camera TF, and its position can be changed at runtime, like the robot moving between acquisition poses.

This requires the [`surface_reconstruction`](https://github.com/JRL-CARI-CNR-UNIBS/surface_reconstruction) and `surface_reconstruction_msgs` packages in the same workspace.

Keep terminals 1 and 2 running. **Terminal 4**, node 2:

```bash
ros2 launch surface_reconstruction cloud_merger.launch.py
```

**Terminal 5**, four acquisitions:

```bash
ros2 param set /fake_realsense cam_x -0.15
ros2 param set /fake_realsense cam_y -0.10
ros2 service call /cloud_merger_node/acquire std_srvs/srv/Trigger

ros2 param set /fake_realsense cam_x 0.15
ros2 service call /cloud_merger_node/acquire std_srvs/srv/Trigger

ros2 param set /fake_realsense cam_y 0.10
ros2 service call /cloud_merger_node/acquire std_srvs/srv/Trigger

ros2 param set /fake_realsense cam_x -0.15
ros2 service call /cloud_merger_node/acquire std_srvs/srv/Trigger
```

Each call answers something like:

```
response: success=True, message='Acquisition #1 stored (50406 points)'
```

Then reconstruct the surface:

```bash
ros2 service call /cloud_merger_node/compute_surface std_srvs/srv/Trigger
```

```
response: success=True, message='Surface computed from 4 acquisitions: 200790 merged points,
  23144 after filtering, ... mesh with 53271 vertices and 106186 triangles (4.0 s)'
```

In Rviz, add `/cloud_merger_node/mesh_marker` (Marker, *Durability Policy* = Transient Local). The mesh covers only the object: the cable never reached node 2, and node 2 cut the table.

To see why node 1 is needed, disable the clustering and repeat the scan:

```bash
ros2 param set /cloud_filter_node cluster_enabled false
ros2 service call /cloud_merger_node/reset std_srvs/srv/Trigger
# ... four acquisitions and compute_surface as above
```

The cable ends up in the mesh, as a surface floating 15–20 cm above the table. The object also becomes coarser: the Poisson octree now has to cover the cable as well, so at the same `poisson_depth` its cells are larger (in our test, 27516 vertices instead of 53271).

Wait about one second after moving the camera before calling `acquire`. The service takes the latest filtered cloud, and right after a `param set` that cloud may still come from the previous pose. On the robot, the same rule applies: acquire only when the robot is still.

---

## 3. Use with the RealSense

### 3.1 Camera driver

Start the driver with the point cloud enabled:

```bash
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true
```

and find the cloud topic:

```bash
ros2 topic list | grep points
```

With the driver defaults it is `/camera/camera/depth/color/points`. In the cell the camera is named `wrist`, and the topic is `/wrist/depth/color/points`, which is the default of the launch file. The cloud is in the depth optical frame of the camera (z forward, x right, y down). Node 1 works in that frame, so it does not need the TF.

Optional: the driver has its own depth post-processing filters (in realsense-ros 4.x, e.g. `spatial_filter.enable:=true`, `temporal_filter.enable:=true`). They work on the depth image, before the cloud is built, and reduce the noise that reaches node 1. The temporal filter averages over several frames: after the robot stops, wait a little longer before `acquire`.

### 3.2 Configuration

Copy the parameter file and adapt it:

```bash
cp $(ros2 pkg prefix surface_acquisition)/share/surface_acquisition/config/cloud_filter.yaml ~/my_filter.yaml
```

Check these first:

- `min_depth`: just beyond the gripper or the tool, if they are in the field of view. The D435 does not measure closer than about 0.1–0.3 m (it depends on the resolution) anyway.
- `max_depth`: a little beyond the farthest point of the table seen from the highest acquisition pose. With the camera tilted, the far edge of the table is much farther than the centre.
- `x_min` … `y_max`: only if something to the side of the table survives the crop and is larger than the table (e.g. a nearby workbench).
- `voxel_leaf`: 2–4 mm for a camera at 40–50 cm. This sets the point spacing seen by node 2 (see node 2's tuning rules).

### 3.3 Start-up

```bash
ros2 launch surface_acquisition cloud_filter.launch.py \
    input_cloud:=/wrist/depth/color/points \
    params_file:=$HOME/my_filter.yaml
```

Show `/filtered_cloud` in Rviz with the camera frame or `world` as fixed frame, move the robot over the object and check that the output always contains the table and the object, and nothing else.

---

## 4. Parameter tuning and common problems

### Rules of thumb

| Parameter | Suggested value |
|---|---|
| `voxel_leaf` | 2–4 mm at 40–50 cm; larger if the processing is too slow |
| `cluster_tolerance` | ≈ 3 · `voxel_leaf`; larger if the object is split into several clusters, smaller if foreign objects stick to it |
| `cluster_min_size` | a few hundred points: only relevant for the cluster count in the log |
| `sor_mean_k` | 20–50; larger values are more robust but slower |
| `sor_stddev_mul` | 1.0; decrease it to remove more noise at the edges, at the risk of eating the borders of the object |
| `normal_k_search` | 15–30; with a large `voxel_leaf` use fewer neighbours, or `normal_radius` ≈ 3 · `voxel_leaf` |

Processing time grows roughly linearly with the number of points after the voxel grid. The log shows which stage takes the longest.

### Common problems

| Symptom | Likely cause | Fix |
|---|---|---|
| No log summary after start-up | wrong input topic | `ros2 topic list \| grep points`; launch with `input_cloud:=...` |
| `Empty output: ... 0 cropped` | depth limits exclude everything | check `min_depth` / `max_depth` against the camera–table distance |
| `Empty output: ... 0 clusters` | every cluster is smaller than `cluster_min_size` or larger than `cluster_max_size` | lower `cluster_min_size`, raise `cluster_max_size` |
| Output contains the floor or a nearby object instead of the table | the other surface is the largest cluster | lower `max_depth`, or use the `x` / `y` limits |
| Object and table split, only one of them in the output | `cluster_tolerance` too small for the point spacing (dark edges, steep sides with few points) | increase `cluster_tolerance` |
| A cable or the robot cable stuck to the object | `cluster_tolerance` too large, or it actually touches the table | decrease `cluster_tolerance`; route the cable away |
| Fringes of points at the edges of the object | RealSense flying pixels at depth discontinuities | lower `sor_stddev_mul`; enable the driver's spatial filter |
| Low rate in the log | too many points | increase `voxel_leaf`; reduce `max_depth`; build in `Release` |
| `ros2 topic hz /filtered_cloud` reports a much lower rate than the log | `ros2 topic hz` is written in Python and cannot keep up with large clouds | trust the rate in the node log |
| `acquire` in node 2 says the cloud is too old | node 1 too slow (or stopped), or clocks not synchronised | check the rate in the log; see node 2's tutorial |
| `Setting parameter failed: Wrong parameter type` | integer value for a double parameter | write `0.0`, not `0` |
