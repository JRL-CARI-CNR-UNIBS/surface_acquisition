// Copyright 2026 CARI-JRL
// SPDX-License-Identifier: Apache-2.0
// Author: Manuel Beschi <manuel.beschi@unibs.it>

#pragma once

#include <cstddef>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace surface_acquisition
{

using Point = pcl::PointXYZ;
using Cloud = pcl::PointCloud<Point>;
using PointN = pcl::PointNormal;
using CloudN = pcl::PointCloud<PointN>;

/// Tuning of the filtering pipeline (node parameters). All lengths in metres,
/// expressed in the sensor frame (z = depth along the optical axis).
struct FilterOptions
{
  // 1. Crop (PassThrough). The z limits are always applied; the x / y limits
  //    only when min < max.
  double min_depth{0.15};
  double max_depth{1.0};
  double x_min{0.0};
  double x_max{0.0};
  double y_min{0.0};
  double y_max{0.0};

  // 2. Voxel grid; <= 0 disables
  double voxel_leaf{0.003};

  // 3. Statistical outlier removal; sor_mean_k <= 0 disables
  int sor_mean_k{30};
  double sor_stddev_mul{1.0};

  // 4. Euclidean cluster extraction: only the largest cluster is kept
  bool cluster_enabled{true};
  double cluster_tolerance{0.01};
  int cluster_min_size{100};
  int cluster_max_size{10000000};

  // 5. Normal estimation: radius search if normal_radius > 0, k-nearest otherwise
  int normal_k_search{20};
  double normal_radius{0.0};
  int threads{0};  ///< OpenMP threads for the normals, 0 = automatic
};

/// Returns an empty string if the options are consistent, the reason otherwise.
std::string validate(const FilterOptions & o);

/// Number of points after every stage of the pipeline.
struct FilterStats
{
  std::size_t input{0};       ///< input points, invalid (NaN) ones included
  std::size_t cropped{0};     ///< after the PassThrough crop (invalid points dropped)
  std::size_t downsampled{0}; ///< after the voxel grid
  std::size_t denoised{0};    ///< after the statistical outlier removal
  std::size_t clusters{0};    ///< clusters found (0 if clustering is disabled)
  std::size_t clustered{0};   ///< points of the selected cluster
  std::size_t output{0};      ///< output points with a valid normal

  // Time spent in every stage [ms]
  double t_crop{0.0};
  double t_voxel{0.0};
  double t_sor{0.0};
  double t_cluster{0.0};
  double t_normals{0.0};
};

/// Output of filterCloud.
struct FilterResult
{
  CloudN::Ptr cloud;             ///< largest cluster with normals, in the input frame
  Cloud::Ptr before_clustering;  ///< cloud after crop + voxel grid + SOR (for tuning)
  FilterStats stats;
};

/**
 * Filtering pipeline of node 1, independent of ROS:
 *   PassThrough crop -> voxel grid -> statistical outlier removal ->
 *   Euclidean clustering (largest cluster) -> normal estimation.
 *
 * The crop comes before the voxel grid: it is cheaper (one comparison per point)
 * and it bounds the extent of the cloud, so the voxel grid does not overflow its
 * index range with small leaves.
 *
 * The normals are computed last, on the selected cluster only, and are oriented
 * towards the sensor (origin of the input frame).
 */
FilterResult filterCloud(const Cloud::ConstPtr & input, const FilterOptions & o);

}  // namespace surface_acquisition
