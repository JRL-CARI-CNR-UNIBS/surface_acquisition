// Copyright 2026 CARI-JRL
// SPDX-License-Identifier: Apache-2.0
// Author: Manuel Beschi <manuel.beschi@unibs.it>

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "surface_acquisition/cloud_filter_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Creare a multi-threaded executor to allow the parameter callback to run concurrently with the subscription callback
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(std::make_shared<surface_acquisition::CloudFilterNode>());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
