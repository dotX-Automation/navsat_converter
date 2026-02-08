/**
 * NavSat Converter node validators.
 *
 * Giorgio Manca <giorgio.manca.97@gmail.com>
 *
 * July 8, 2025
 */

/**
 * Copyright 2025 dotX Automation s.r.l.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <navsat_converter/navsat_converter.hpp>

namespace navsat_converter
{


bool NavSatConverter::validate_conv_gps_fix_topics(const rclcpp::Parameter & p)
{
  std::vector<std::string> list = p.as_string_array();

  conv_gps_fix_topics_.clear();
  for (const std::string & topic : list) {
    if (topic.length() > 0) {
      conv_gps_fix_topics_.push_back(topic);
    }
  }

  return true;
}


bool NavSatConverter::validate_conv_xyz_odometry_topics(const rclcpp::Parameter & p)
{
  std::vector<std::string> list = p.as_string_array();

  conv_xyz_odometry_topics_.clear();
  for (const std::string & topic : list) {
    if (topic.length() > 0) {
      conv_xyz_odometry_topics_.push_back(topic);
    }
  }

  return true;
}


bool NavSatConverter::validate_conv_xyz_point_topics(const rclcpp::Parameter & p)
{
  std::vector<std::string> list = p.as_string_array();

  conv_xyz_point_topics_.clear();
  for (const std::string & topic : list) {
    if (topic.length() > 0) {
      conv_xyz_point_topics_.push_back(topic);
    }
  }

  return true;
}


bool NavSatConverter::validate_conv_xyz_pose_topics(const rclcpp::Parameter & p)
{
  std::vector<std::string> list = p.as_string_array();

  conv_xyz_pose_topics_.clear();
  for (const std::string & topic : list) {
    if (topic.length() > 0) {
      conv_xyz_pose_topics_.push_back(topic);
    }
  }

  return true;
}


bool NavSatConverter::validate_conv_xyz_posecov_topics(const rclcpp::Parameter & p)
{
  std::vector<std::string> list = p.as_string_array();

  conv_xyz_posecov_topics_.clear();
  for (const std::string & topic : list) {
    if (topic.length() > 0) {
      conv_xyz_posecov_topics_.push_back(topic);
    }
  }

  return true;
}


} // namespace navsat_converter
