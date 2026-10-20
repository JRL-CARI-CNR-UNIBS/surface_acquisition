// Copyright 2026 CARI-JRL
// SPDX-License-Identifier: Apache-2.0
// Author: Manuel Beschi <manuel.beschi@unibs.it>

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "surface_acquisition/cloud_filter_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<surface_acquisition::CloudFilterNode>());
  rclcpp::shutdown();
  return 0;
}
