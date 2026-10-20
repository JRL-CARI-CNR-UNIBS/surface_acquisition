// Copyright 2026 CARI-JRL
// SPDX-License-Identifier: Apache-2.0
// Author: Manuel Beschi <manuel.beschi@unibs.it>

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "surface_acquisition/cloud_filter_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Keep a reference: executors only hold a weak pointer to the node, so a
  // temporary passed to add_node() is destroyed immediately.
  auto node = std::make_shared<surface_acquisition::CloudFilterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
