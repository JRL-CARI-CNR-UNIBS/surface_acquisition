// Copyright 2026 CARI-JRL
// SPDX-License-Identifier: Apache-2.0
// Author: Manuel Beschi <manuel.beschi@unibs.it>

#include "surface_acquisition/cloud_filter.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <numeric>
#include <unordered_map>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

#include <pcl/common/io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>

namespace surface_acquisition
{

namespace
{

Cloud::Ptr passThrough(const Cloud::ConstPtr & in, const char * field, double min, double max)
{
  pcl::PassThrough<Point> pass;
  pass.setInputCloud(in);
  pass.setFilterFieldName(field);
  pass.setFilterLimits(static_cast<float>(min), static_cast<float>(max));
  auto out = std::make_shared<Cloud>();
  pass.filter(*out);
  return out;
}

/// Milliseconds elapsed since t, which is then reset to now.
double lap(std::chrono::steady_clock::time_point & t)
{
  const auto now = std::chrono::steady_clock::now();
  const double ms = std::chrono::duration<double, std::milli>(now - t).count();
  t = now;
  return ms;
}

int numThreads(int requested)
{
#ifdef _OPENMP
  return requested > 0 ? requested : omp_get_max_threads();
#else
  (void)requested;
  return 1;
#endif
}

/**
 * Same criterion as pcl::StatisticalOutlierRemoval: a point is kept if the mean
 * distance from its k nearest neighbours is below mean + stddev_mul * stddev,
 * computed over the whole cloud. The neighbour searches run in parallel.
 */
Cloud::Ptr removeStatisticalOutliers(
  const Cloud::ConstPtr & cloud, int k, double stddev_mul, int threads)
{
  pcl::search::KdTree<Point> tree;
  tree.setInputCloud(cloud);
  const auto n = static_cast<std::ptrdiff_t>(cloud->size());

  std::vector<double> mean_dist(cloud->size(), 0.0);
  std::vector<char> valid(cloud->size(), 0);
#pragma omp parallel num_threads(threads)
  {
    pcl::Indices idx(k + 1);
    std::vector<float> d2(k + 1);
#pragma omp for schedule(dynamic, 256)
    for (std::ptrdiff_t i = 0; i < n; ++i) {
      // The first neighbour is the point itself
      if (tree.nearestKSearch(static_cast<pcl::index_t>(i), k + 1, idx, d2) != k + 1) {
        continue;
      }
      double sum = 0.0;
      for (int j = 1; j <= k; ++j) {
        sum += std::sqrt(d2[j]);
      }
      mean_dist[i] = sum / k;
      valid[i] = 1;
    }
  }

  double sum = 0.0, sq_sum = 0.0;
  std::size_t n_valid = 0;
  for (std::size_t i = 0; i < mean_dist.size(); ++i) {
    if (valid[i]) {
      sum += mean_dist[i];
      sq_sum += mean_dist[i] * mean_dist[i];
      ++n_valid;
    }
  }
  auto out = std::make_shared<Cloud>();
  if (n_valid < 2) {
    return out;
  }
  const double mean = sum / n_valid;
  const double variance = std::max(0.0, (sq_sum - sum * sum / n_valid) / (n_valid - 1));
  const double threshold = mean + stddev_mul * std::sqrt(variance);

  out->header = cloud->header;
  out->reserve(n_valid);
  for (std::size_t i = 0; i < mean_dist.size(); ++i) {
    if (valid[i] && mean_dist[i] <= threshold) {
      out->push_back((*cloud)[i]);
    }
  }
  return out;
}

/// Union-find with path halving.
std::size_t findRoot(std::vector<std::size_t> & parent, std::size_t x)
{
  while (parent[x] != x) {
    parent[x] = parent[parent[x]];
    x = parent[x];
  }
  return x;
}

/**
 * Same clusters as pcl::EuclideanClusterExtraction (connected components of the
 * graph linking points closer than tolerance, kept if their size is within
 * [min_size, max_size]), but returns only the largest one. The radius searches
 * run in parallel, block by block, to bound the memory used by the neighbour
 * lists; the components are merged serially with a union-find.
 */
pcl::Indices largestEuclideanCluster(
  const Cloud::ConstPtr & cloud, double tolerance, int min_size, int max_size, int threads,
  std::size_t & n_clusters)
{
  pcl::search::KdTree<Point> tree(false);  // unsorted results: faster radius search
  tree.setInputCloud(cloud);
  const std::size_t n = cloud->size();

  std::vector<std::size_t> parent(n);
  std::iota(parent.begin(), parent.end(), 0);

  constexpr std::size_t block = 8192;
  std::vector<pcl::Indices> neighbours(std::min(n, block));
  for (std::size_t start = 0; start < n; start += block) {
    const auto end = static_cast<std::ptrdiff_t>(std::min(n, start + block));
#pragma omp parallel num_threads(threads)
    {
      std::vector<float> d2;
#pragma omp for schedule(dynamic, 64)
      for (auto i = static_cast<std::ptrdiff_t>(start); i < end; ++i) {
        tree.radiusSearch(static_cast<pcl::index_t>(i), tolerance, neighbours[i - start], d2);
      }
    }
    for (auto i = static_cast<std::size_t>(start); i < static_cast<std::size_t>(end); ++i) {
      for (const auto j : neighbours[i - start]) {
        const std::size_t a = findRoot(parent, i), b = findRoot(parent, static_cast<std::size_t>(j));
        if (a != b) {
          parent[std::max(a, b)] = std::min(a, b);
        }
      }
    }
  }

  std::unordered_map<std::size_t, std::size_t> sizes;
  for (std::size_t i = 0; i < n; ++i) {
    ++sizes[findRoot(parent, i)];
  }
  n_clusters = 0;
  std::size_t best_root = n, best_size = 0;
  for (const auto & [root, size] : sizes) {
    if (size < static_cast<std::size_t>(min_size) || size > static_cast<std::size_t>(max_size)) {
      continue;
    }
    ++n_clusters;
    if (size > best_size) {
      best_root = root;
      best_size = size;
    }
  }

  pcl::Indices largest;
  largest.reserve(best_size);
  for (std::size_t i = 0; i < n && best_size > 0; ++i) {
    if (parent[i] == best_root) {  // fully compressed by the counting loop above
      largest.push_back(static_cast<pcl::index_t>(i));
    }
  }
  return largest;
}

bool isFinite(const PointN & p)
{
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
         std::isfinite(p.normal_x) && std::isfinite(p.normal_y) && std::isfinite(p.normal_z);
}

}  // namespace

std::string validate(const FilterOptions & o)
{
  if (!(o.min_depth < o.max_depth)) {
    return "min_depth must be smaller than max_depth";
  }
  if (o.sor_mean_k > 0 && o.sor_stddev_mul <= 0.0) {
    return "sor_stddev_mul must be positive";
  }
  if (o.cluster_enabled) {
    if (o.cluster_tolerance <= 0.0) {
      return "cluster_tolerance must be positive";
    }
    if (o.cluster_min_size < 1 || o.cluster_max_size < o.cluster_min_size) {
      return "cluster sizes must satisfy 1 <= cluster_min_size <= cluster_max_size";
    }
    if (o.voxel_leaf > 0.0 && o.cluster_tolerance <= o.voxel_leaf) {
      return "cluster_tolerance must be larger than voxel_leaf, "
             "otherwise neighbouring voxels are split into different clusters";
    }
  }
  if (o.normal_radius <= 0.0 && o.normal_k_search < 3) {
    return "normal_k_search must be at least 3 (or set normal_radius > 0)";
  }
  if (o.threads < 0) {
    return "threads must be >= 0";
  }
  return "";
}

FilterResult filterCloud(const Cloud::ConstPtr & input, const FilterOptions & o)
{
  FilterResult r;
  r.cloud = std::make_shared<CloudN>();
  r.cloud->header = input->header;
  r.stats.input = input->size();
  auto t = std::chrono::steady_clock::now();

  // 1. Crop on the depth (and optionally on x / y). PassThrough also drops the
  //    NaN points the RealSense returns where the depth is invalid.
  Cloud::Ptr cloud = passThrough(input, "z", o.min_depth, o.max_depth);
  if (o.x_min < o.x_max) {
    cloud = passThrough(cloud, "x", o.x_min, o.x_max);
  }
  if (o.y_min < o.y_max) {
    cloud = passThrough(cloud, "y", o.y_min, o.y_max);
  }
  r.stats.cropped = cloud->size();
  r.stats.t_crop = lap(t);

  // 2. Voxel grid: lighter cloud and uniform density
  if (o.voxel_leaf > 0.0 && !cloud->empty()) {
    pcl::VoxelGrid<Point> vg;
    vg.setInputCloud(cloud);
    const auto leaf = static_cast<float>(o.voxel_leaf);
    vg.setLeafSize(leaf, leaf, leaf);
    auto tmp = std::make_shared<Cloud>();
    vg.filter(*tmp);
    cloud = tmp;
  }
  r.stats.downsampled = cloud->size();
  r.stats.t_voxel = lap(t);

  // 3. Statistical outlier removal: residual flying pixels and speckles
  const int threads = numThreads(o.threads);
  if (o.sor_mean_k > 0 && cloud->size() > static_cast<std::size_t>(o.sor_mean_k)) {
    cloud = removeStatisticalOutliers(cloud, o.sor_mean_k, o.sor_stddev_mul, threads);
  }
  r.stats.denoised = cloud->size();
  r.stats.t_sor = lap(t);
  r.before_clustering = cloud;

  // 4. Euclidean clustering, keeping the cluster with the most points
  if (o.cluster_enabled && !cloud->empty()) {
    const pcl::Indices largest = largestEuclideanCluster(
      cloud, o.cluster_tolerance, o.cluster_min_size, o.cluster_max_size, threads,
      r.stats.clusters);
    auto tmp = std::make_shared<Cloud>();
    pcl::copyPointCloud(*cloud, largest, *tmp);
    cloud = tmp;
  }
  r.stats.clustered = cloud->size();
  r.stats.t_cluster = lap(t);

  // 5. Normals, on the final cluster only, oriented towards the sensor (origin of
  //    the input frame). Points without enough neighbours get NaN normals and are
  //    dropped.
  if (cloud->size() >= 3) {
    auto tree = std::make_shared<pcl::search::KdTree<Point>>();
    tree->setInputCloud(cloud);
    pcl::NormalEstimationOMP<Point, pcl::Normal> ne;
    ne.setNumberOfThreads(static_cast<unsigned int>(threads));
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    if (o.normal_radius > 0.0) {
      ne.setRadiusSearch(o.normal_radius);
    } else {
      ne.setKSearch(o.normal_k_search);
    }
    ne.setViewPoint(0.0f, 0.0f, 0.0f);
    pcl::PointCloud<pcl::Normal> normals;
    ne.compute(normals);

    CloudN with_normals;
    pcl::concatenateFields(*cloud, normals, with_normals);
    r.cloud->reserve(with_normals.size());
    for (const auto & p : with_normals.points) {
      if (isFinite(p)) {
        r.cloud->push_back(p);
      }
    }
  }
  r.cloud->header = input->header;
  r.cloud->is_dense = true;
  r.stats.output = r.cloud->size();
  r.stats.t_normals = lap(t);
  return r;
}

}  // namespace surface_acquisition
