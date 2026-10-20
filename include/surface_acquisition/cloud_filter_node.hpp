// Copyright 2026 CARI-JRL
// SPDX-License-Identifier: Apache-2.0
// Author: Manuel Beschi <manuel.beschi@unibs.it>

#pragma once

#include <chrono>
#include <cstddef>
#include <vector>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "surface_acquisition/cloud_filter.hpp"

namespace surface_acquisition
{

/**
 * Node 1 of the pipeline (live acquisition): continuously filters the camera
 * cloud and publishes the largest cluster with normals, for Rviz and for the
 * acquire service of node 2 (cloud_merger_node).
 *
 * Parameters can be changed at runtime; invalid combinations are rejected.
 */
class CloudFilterNode : public rclcpp::Node
{
public:
  /**
   * @brief Declare the parameters, subscribe to the camera cloud and create the publishers.
   * @param options Node options.
   */
  explicit CloudFilterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /**
   * @brief Declare the parameters and read the initial filter options.
   */
  void declareParameters();
  /**
   * @brief Validate and apply runtime parameter changes; invalid combinations are rejected.
   */
  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & parameters);

  /**
   * @brief Filter one camera cloud (rate-limited by max_rate) and publish the result.
   */
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  // Current configuration. Parameters are only changed by service callbacks of
  // this node, executed by the same single-threaded executor as the
  // subscription, so no locking is needed.
  FilterOptions options_;
  double max_rate_{10.0};
  bool publish_debug_{false};

  std::chrono::steady_clock::time_point last_processed_;
  std::chrono::steady_clock::time_point last_log_{std::chrono::steady_clock::now()};
  std::size_t frames_since_log_{0};

  // Heartbeat timer to log that the node is alive, even if no clouds are received
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

}  // namespace surface_acquisition
