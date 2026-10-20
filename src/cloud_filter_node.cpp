// Copyright 2026 CARI-JRL
// SPDX-License-Identifier: Apache-2.0
// Author: Manuel Beschi <manuel.beschi@unibs.it>

#include "surface_acquisition/cloud_filter_node.hpp"

#include <memory>
#include <stdexcept>
#include <string>

#include <pcl_conversions/pcl_conversions.h>

namespace surface_acquisition
{

namespace
{

/// Copies a parameter into the matching FilterOptions field.
/// Returns false if the name does not belong to the filter.
bool applyParameter(FilterOptions & o, const rclcpp::Parameter & p)
{
  const std::string & n = p.get_name();
  if (n == "min_depth") {
    o.min_depth = p.as_double();
  } else if (n == "max_depth") {
    o.max_depth = p.as_double();
  } else if (n == "x_min") {
    o.x_min = p.as_double();
  } else if (n == "x_max") {
    o.x_max = p.as_double();
  } else if (n == "y_min") {
    o.y_min = p.as_double();
  } else if (n == "y_max") {
    o.y_max = p.as_double();
  } else if (n == "voxel_leaf") {
    o.voxel_leaf = p.as_double();
  } else if (n == "sor_mean_k") {
    o.sor_mean_k = static_cast<int>(p.as_int());
  } else if (n == "sor_stddev_mul") {
    o.sor_stddev_mul = p.as_double();
  } else if (n == "cluster_enabled") {
    o.cluster_enabled = p.as_bool();
  } else if (n == "cluster_tolerance") {
    o.cluster_tolerance = p.as_double();
  } else if (n == "cluster_min_size") {
    o.cluster_min_size = static_cast<int>(p.as_int());
  } else if (n == "cluster_max_size") {
    o.cluster_max_size = static_cast<int>(p.as_int());
  } else if (n == "normal_k_search") {
    o.normal_k_search = static_cast<int>(p.as_int());
  } else if (n == "normal_radius") {
    o.normal_radius = p.as_double();
  } else if (n == "threads") {
    o.threads = static_cast<int>(p.as_int());
  } else {
    return false;
  }
  return true;
}

}  // namespace

CloudFilterNode::CloudFilterNode(const rclcpp::NodeOptions & options)
: Node("cloud_filter_node", options)
{
  declareParameters();

  // Initial configuration, validated like any later change
  for (const auto & p : get_parameters(list_parameters({}, 1).names)) {
    applyParameter(options_, p);
  }
  const std::string error = validate(options_);
  if (!error.empty()) {
    RCLCPP_FATAL(get_logger(), "Invalid parameters: %s", error.c_str());
    throw std::invalid_argument(error);
  }
  max_rate_ = get_parameter("max_rate").as_double();
  publish_debug_ = get_parameter("publish_debug").as_bool();
  param_cb_ = add_on_set_parameters_callback(
    std::bind(&CloudFilterNode::onSetParameters, this, std::placeholders::_1));


  // Reliable, so that Rviz (reliable by default) and node 2 (best effort) can both subscribe
  filtered_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("filtered_cloud", rclcpp::QoS(5));
  debug_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/before_clustering", rclcpp::QoS(5));


  // Only the most recent camera frame is of interest: with depth 1 the frames that
  // arrive while a cloud is being processed are overwritten instead of queued.
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "input_cloud", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&CloudFilterNode::cloudCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(), "cloud_filter_node ready: '%s' -> '%s'",
    cloud_sub_->get_topic_name(), filtered_pub_->get_topic_name());


  // add a heartbeat timer to log that the node is alive, even if no clouds are received, 20 seconds
  heartbeat_timer_ = create_wall_timer(
    std::chrono::seconds(20), [this]() {
      RCLCPP_INFO(get_logger(), "cloud_filter_node alive: '%s' -> '%s'", cloud_sub_->get_topic_name(), filtered_pub_->get_topic_name());
    });
}

void CloudFilterNode::declareParameters()
{
  const FilterOptions d;

  // Crop in the sensor frame (z = depth); x / y limits disabled when min >= max
  declare_parameter<double>("min_depth", d.min_depth);
  declare_parameter<double>("max_depth", d.max_depth);
  declare_parameter<double>("x_min", d.x_min);
  declare_parameter<double>("x_max", d.x_max);
  declare_parameter<double>("y_min", d.y_min);
  declare_parameter<double>("y_max", d.y_max);

  // Voxel grid, <= 0 disables
  declare_parameter<double>("voxel_leaf", d.voxel_leaf);

  // Statistical outlier removal, sor_mean_k <= 0 disables
  declare_parameter<int>("sor_mean_k", d.sor_mean_k);
  declare_parameter<double>("sor_stddev_mul", d.sor_stddev_mul);

  // Euclidean clustering (largest cluster kept)
  declare_parameter<bool>("cluster_enabled", d.cluster_enabled);
  declare_parameter<double>("cluster_tolerance", d.cluster_tolerance);
  declare_parameter<int>("cluster_min_size", d.cluster_min_size);
  declare_parameter<int>("cluster_max_size", d.cluster_max_size);

  // Normals: radius search if normal_radius > 0, k-nearest otherwise
  declare_parameter<int>("normal_k_search", d.normal_k_search);
  declare_parameter<double>("normal_radius", d.normal_radius);
  declare_parameter<int>("threads", d.threads);

  // Node behaviour
  declare_parameter<double>("max_rate", 10.0);         // [Hz], <= 0 processes every frame
  declare_parameter<bool>("publish_debug", false);     // publish ~/before_clustering
}

rcl_interfaces::msg::SetParametersResult CloudFilterNode::onSetParameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  // Apply the whole change set to a copy, then accept it only if consistent
  FilterOptions candidate = options_;
  double max_rate = max_rate_;
  bool publish_debug = publish_debug_;
  for (const auto & p : parameters) {
    if (applyParameter(candidate, p)) {
      continue;
    }
    if (p.get_name() == "max_rate") {
      max_rate = p.as_double();
    } else if (p.get_name() == "publish_debug") {
      publish_debug = p.as_bool();
    }
  }

  result.reason = validate(candidate);
  if (!result.reason.empty()) {
    result.successful = false;
    RCLCPP_WARN(get_logger(), "Parameter change rejected: %s", result.reason.c_str());
    return result;
  }
  options_ = candidate;
  max_rate_ = max_rate;
  publish_debug_ = publish_debug;
  return result;
}

void CloudFilterNode::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  RCLCPP_ERROR(get_logger(), "Received cloud with %u points", msg->width * msg->height);
  const auto t_start = std::chrono::steady_clock::now();
  if (max_rate_ > 0.0 &&
    std::chrono::duration<double>(t_start - last_processed_).count() < 1.0 / max_rate_)
  {
    RCLCPP_WARN(get_logger(), "Skipping cloud to respect max_rate = %.1f Hz", max_rate_);
    return;
  }
  if (processing_) {
    RCLCPP_WARN(get_logger(), "Skipping cloud because previous one is still being processed");
    return;
  }
  processing_ = true;
  last_processed_ = t_start;

  auto input = std::make_shared<Cloud>();
  pcl::fromROSMsg(*msg, *input);

  RCLCPP_ERROR(get_logger(), "Converted cloud with %zu points", input->size());

  const FilterResult r = filterCloud(input, options_);

  // The output keeps the input header: same frame, and same stamp, so that node 2
  // looks up the camera pose at the instant the frame was captured. An empty cloud
  // is published too, so that Rviz and node 2 never keep a stale result.
  sensor_msgs::msg::PointCloud2 out;
  pcl::toROSMsg(*r.cloud, out);
  out.header = msg->header;
  filtered_pub_->publish(out);

  if (publish_debug_ && debug_pub_->get_subscription_count() > 0) {
    sensor_msgs::msg::PointCloud2 dbg;
    pcl::toROSMsg(*r.before_clustering, dbg);
    dbg.header = msg->header;
    debug_pub_->publish(dbg);
  }

  RCLCPP_ERROR(get_logger(), "Published cloud with %zu points", r.cloud->size());

  const double ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_start).count();
  const auto & s = r.stats;
  if (s.output == 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Empty output: %zu input, %zu cropped, %zu downsampled, %zu denoised, %zu clusters",
      s.input, s.cropped, s.downsampled, s.denoised, s.clusters);
  }

  // Periodic summary, with the actual processing rate
  ++frames_since_log_;
  const double since_log = std::chrono::duration<double>(t_start - last_log_).count();
  if (since_log >= 5.0) {
    const std::string clusters = options_.cluster_enabled ?
      "largest of " + std::to_string(s.clusters) + " clusters" : "clustering disabled";
    RCLCPP_INFO(
      get_logger(),
      "%.1f Hz | points: %zu input -> %zu crop -> %zu voxel -> %zu SOR -> %zu (%s) -> %zu with "
      "normals | %.0f ms (crop %.0f, voxel %.0f, SOR %.0f, cluster %.0f, normals %.0f)",
      frames_since_log_ / since_log, s.input, s.cropped, s.downsampled, s.denoised, s.clustered,
      clusters.c_str(), s.output, ms, s.t_crop, s.t_voxel, s.t_sor, s.t_cluster, s.t_normals);
    frames_since_log_ = 0;
    last_log_ = t_start;
  }

  RCLCPP_FATAL(get_logger(), "Cloud filtering done in %.0f ms, %zu points output", ms, s.output);
}

}  // namespace surface_acquisition
