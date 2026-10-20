// Copyright 2026 CARI-JRL
// SPDX-License-Identifier: Apache-2.0
// Author: Manuel Beschi <manuel.beschi@unibs.it>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <random>

#include "surface_acquisition/cloud_filter.hpp"

using surface_acquisition::Cloud;
using surface_acquisition::FilterOptions;
using surface_acquisition::FilterResult;
using surface_acquisition::Point;
using surface_acquisition::filterCloud;
using surface_acquisition::validate;

namespace
{

/// Square patch in the plane z = depth, centred in (cx, cy), seen by a camera in the origin.
void addPatch(Cloud & c, double cx, double cy, double depth, double side, double res)
{
  const int n = static_cast<int>(std::round(side / res));
  for (int i = 0; i <= n; ++i) {
    for (int j = 0; j <= n; ++j) {
      c.push_back(Point(
          static_cast<float>(cx - side / 2 + i * res),
          static_cast<float>(cy - side / 2 + j * res),
          static_cast<float>(depth)));
    }
  }
}

/// Options with every optional stage disabled.
FilterOptions bareOptions()
{
  FilterOptions o;
  o.min_depth = 0.0;
  o.max_depth = 10.0;
  o.voxel_leaf = 0.0;
  o.sor_mean_k = 0;
  o.cluster_enabled = false;
  return o;
}

}  // namespace

TEST(Validate, DefaultsAreValid)
{
  EXPECT_TRUE(validate(FilterOptions()).empty());
}

TEST(Validate, RejectsInconsistentOptions)
{
  FilterOptions o;
  o.min_depth = 1.0;
  o.max_depth = 0.5;
  EXPECT_FALSE(validate(o).empty());

  o = FilterOptions();
  o.cluster_tolerance = o.voxel_leaf;  // neighbouring voxels would be split
  EXPECT_FALSE(validate(o).empty());

  o = FilterOptions();
  o.cluster_min_size = 10;
  o.cluster_max_size = 5;
  EXPECT_FALSE(validate(o).empty());

  o = FilterOptions();
  o.normal_k_search = 2;
  EXPECT_FALSE(validate(o).empty());
  o.normal_radius = 0.01;  // radius search: k is not used
  EXPECT_TRUE(validate(o).empty());

  o = FilterOptions();
  o.cluster_enabled = false;
  o.cluster_tolerance = 0.0;  // ignored when clustering is disabled
  EXPECT_TRUE(validate(o).empty());
}

TEST(FilterCloud, EmptyInput)
{
  const FilterResult r = filterCloud(std::make_shared<Cloud>(), FilterOptions());
  ASSERT_TRUE(r.cloud);
  EXPECT_TRUE(r.cloud->empty());
  EXPECT_EQ(r.stats.output, 0u);
}

TEST(FilterCloud, CropOnDepthAndLateralLimits)
{
  auto c = std::make_shared<Cloud>();
  addPatch(*c, 0.0, 0.0, 0.05, 0.02, 0.002);   // too close (e.g. the gripper)
  addPatch(*c, 0.0, 0.0, 0.50, 0.10, 0.002);   // object
  addPatch(*c, 0.0, 0.0, 1.50, 0.20, 0.002);   // background
  const float nan = std::numeric_limits<float>::quiet_NaN();
  c->push_back(Point(nan, nan, nan));          // invalid depth

  FilterOptions o = bareOptions();
  o.min_depth = 0.15;
  o.max_depth = 1.0;
  FilterResult r = filterCloud(c, o);
  EXPECT_EQ(r.stats.input, c->size());
  EXPECT_EQ(r.stats.cropped, 51u * 51u);
  for (const auto & p : r.cloud->points) {
    EXPECT_NEAR(p.z, 0.5, 1e-6);
  }

  // Lateral crop: keep x in [-0.001, 0.1] -> columns x = 0, 0.002, ..., 0.05
  o.x_min = -0.001;
  o.x_max = 0.1;
  r = filterCloud(c, o);
  EXPECT_EQ(r.stats.cropped, 26u * 51u);
}

TEST(FilterCloud, VoxelGridReducesDensity)
{
  auto c = std::make_shared<Cloud>();
  addPatch(*c, 0.0, 0.0, 0.5, 0.1, 0.001);

  FilterOptions o = bareOptions();
  o.voxel_leaf = 0.005;
  const FilterResult r = filterCloud(c, o);
  // 0.1 m / 5 mm = 20 voxels per side (+1 for the boundary alignment)
  EXPECT_GE(r.stats.downsampled, 20u * 20u);
  EXPECT_LE(r.stats.downsampled, 22u * 22u);
}

TEST(FilterCloud, OutlierRemovalDropsIsolatedPoints)
{
  auto c = std::make_shared<Cloud>();
  addPatch(*c, 0.0, 0.0, 0.5, 0.1, 0.002);
  const std::size_t n_patch = c->size();
  std::mt19937 gen(42);
  std::uniform_real_distribution<float> u(-0.2f, 0.2f);
  for (int i = 0; i < 20; ++i) {
    c->push_back(Point(u(gen), u(gen), 0.5f + 0.2f + 0.5f * std::abs(u(gen))));
  }

  FilterOptions o = bareOptions();
  o.sor_mean_k = 20;
  o.sor_stddev_mul = 1.0;
  const FilterResult r = filterCloud(c, o);
  EXPECT_LE(r.stats.denoised, n_patch);
  EXPECT_GT(r.stats.denoised, n_patch * 9 / 10);
  for (const auto & p : r.before_clustering->points) {
    EXPECT_NEAR(p.z, 0.5, 1e-6);
  }
}

TEST(FilterCloud, KeepsLargestCluster)
{
  auto c = std::make_shared<Cloud>();
  addPatch(*c, 0.10, 0.0, 0.5, 0.04, 0.002);    // small cluster, first in the cloud
  addPatch(*c, -0.10, 0.0, 0.5, 0.10, 0.002);   // large cluster
  addPatch(*c, 0.0, 0.15, 0.4, 0.06, 0.002);    // medium cluster

  FilterOptions o = bareOptions();
  o.cluster_enabled = true;
  o.cluster_tolerance = 0.005;
  o.cluster_min_size = 10;
  const FilterResult r = filterCloud(c, o);
  EXPECT_EQ(r.stats.clusters, 3u);
  EXPECT_EQ(r.stats.clustered, 51u * 51u);
  for (const auto & p : r.cloud->points) {
    EXPECT_LT(p.x, -0.04);
  }
}

TEST(FilterCloud, NoClusterGivesEmptyOutput)
{
  auto c = std::make_shared<Cloud>();
  addPatch(*c, 0.0, 0.0, 0.5, 0.02, 0.002);

  FilterOptions o = bareOptions();
  o.cluster_enabled = true;
  o.cluster_tolerance = 0.005;
  o.cluster_min_size = 1000;  // larger than the patch
  const FilterResult r = filterCloud(c, o);
  EXPECT_EQ(r.stats.clusters, 0u);
  EXPECT_TRUE(r.cloud->empty());
}

TEST(FilterCloud, NormalsPointTowardsTheSensor)
{
  // Plane z = 0.5 - 0.3 x, tilted with respect to the optical axis
  auto c = std::make_shared<Cloud>();
  for (int i = -25; i <= 25; ++i) {
    for (int j = -25; j <= 25; ++j) {
      const float x = 0.002f * i, y = 0.002f * j;
      c->push_back(Point(x, y, 0.5f - 0.3f * x));
    }
  }

  FilterOptions o;  // full pipeline with the default values
  const FilterResult r = filterCloud(c, o);
  ASSERT_GT(r.stats.output, 100u);
  // Gradient of z + 0.3 x - 0.5 is (0.3, 0, 1): the normal towards the camera is its opposite
  const double nx = -0.3 / std::sqrt(1.09), nz = -1.0 / std::sqrt(1.09);
  for (const auto & p : r.cloud->points) {
    // Towards the origin: n . p < 0
    EXPECT_LT(p.normal_x * p.x + p.normal_y * p.y + p.normal_z * p.z, 0.0f);
    EXPECT_NEAR(p.normal_x, nx, 0.02);
    EXPECT_NEAR(p.normal_y, 0.0, 0.02);
    EXPECT_NEAR(p.normal_z, nz, 0.02);
  }
}

TEST(FilterCloud, KeepsHeader)
{
  auto c = std::make_shared<Cloud>();
  addPatch(*c, 0.0, 0.0, 0.5, 0.1, 0.002);
  c->header.frame_id = "camera_depth_optical_frame";
  c->header.stamp = 123456;
  const FilterResult r = filterCloud(c, FilterOptions());
  EXPECT_EQ(r.cloud->header.frame_id, "camera_depth_optical_frame");
  EXPECT_EQ(r.cloud->header.stamp, 123456u);
}
