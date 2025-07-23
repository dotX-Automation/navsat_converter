/**
 * NavSat Converter node definition.
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

NavSatConverter::NavSatConverter(const rclcpp::NodeOptions & opts)
: NodeBase("navsat_converter", opts, true)
{
  dua_init_node();

  init_internals();

  RCLCPP_INFO(this->get_logger(), "Node initialized");
}


NavSatConverter::~NavSatConverter()
{
  RCLCPP_INFO(this->get_logger(), "Destructor called, sending stop signal.");
}


void NavSatConverter::init_cgroups()
{
  gps_to_xyz_cgroups_fix_.reserve(conv_gps_fix_topics_.size());
  for (size_t i = 0; i < conv_gps_fix_topics_.size(); i++) {
    gps_to_xyz_cgroups_fix_.push_back(dua_create_exclusive_cgroup());
  }

  xyz_to_gps_cgroups_odometry_.reserve(conv_xyz_odometry_topics_.size());
  for (size_t i = 0; i < conv_xyz_odometry_topics_.size(); i++) {
    xyz_to_gps_cgroups_odometry_.push_back(dua_create_exclusive_cgroup());
  }

  xyz_to_gps_cgroups_point_.reserve(conv_xyz_point_topics_.size());
  for (size_t i = 0; i < conv_xyz_point_topics_.size(); i++) {
    xyz_to_gps_cgroups_point_.push_back(dua_create_exclusive_cgroup());
  }

  xyz_to_gps_cgroups_pose_.reserve(conv_xyz_pose_topics_.size());
  for (size_t i = 0; i < conv_xyz_pose_topics_.size(); i++) {
    xyz_to_gps_cgroups_pose_.push_back(dua_create_exclusive_cgroup());
  }

  xyz_to_gps_cgroups_posecov_.reserve(conv_xyz_posecov_topics_.size());
  for (size_t i = 0; i < conv_xyz_posecov_topics_.size(); i++) {
    xyz_to_gps_cgroups_posecov_.push_back(dua_create_exclusive_cgroup());
  }

  convert_gps_cgroup_ = dua_create_exclusive_cgroup();
  convert_xyz_cgroup_ = dua_create_exclusive_cgroup();
  update_earth_cgroup_ = dua_create_exclusive_cgroup();
}


void NavSatConverter::init_subscribers()
{
  if (earth_topic_stream_) {
    coords_sub_ = dua_create_subscription<NavSatFix>(
      coords_topic_,
      std::bind(
        &NavSatConverter::coords_clbk,
        this,
        std::placeholders::_1),
      dua_qos::Reliable::get_datum_qos(),
      coords_cgroup_);
  }

  size_t idx;

  idx = 0;

  gps_to_xyz_subs_fix_.reserve(conv_gps_fix_topics_.size());
  for (size_t i = 0; i < conv_gps_fix_topics_.size(); i++) {
    size_t index = idx;
    gps_to_xyz_subs_fix_.push_back(
      dua_create_subscription<NavSatFix>(
        conv_gps_fix_topics_.at(i),
        [this, index](const NavSatFix::SharedPtr msg) -> void {
          gps_to_xyz_clbk(index, msg);
        },
        dua_qos::Reliable::get_datum_qos(),
        gps_to_xyz_cgroups_fix_.at(i)));
    idx++;
  }

  idx = 0;

  xyz_to_gps_subs_odom_.reserve(conv_xyz_odometry_topics_.size());
  for (size_t i = 0; i < conv_xyz_odometry_topics_.size(); i++) {
    size_t index = idx;
    xyz_to_gps_subs_odom_.push_back(
      dua_create_subscription<Odometry>(
        conv_xyz_odometry_topics_.at(i),
        [this, index](const Odometry::SharedPtr msg) -> void {
          xyz_to_gps_clbk(index, msg);
        },
        dua_qos::Reliable::get_datum_qos(),
        xyz_to_gps_cgroups_odometry_.at(i)));
    idx++;
  }

  xyz_to_gps_subs_point_.reserve(conv_xyz_point_topics_.size());
  for (size_t i = 0; i < conv_xyz_point_topics_.size(); i++) {
    size_t index = idx;
    xyz_to_gps_subs_point_.push_back(
      dua_create_subscription<PointStamped>(
        conv_xyz_point_topics_.at(i),
        [this, index](const PointStamped::SharedPtr msg) -> void {
          xyz_to_gps_clbk(index, msg);
        },
        dua_qos::Reliable::get_datum_qos(),
        xyz_to_gps_cgroups_point_.at(i)));
    idx++;
  }

  xyz_to_gps_subs_pose_.reserve(conv_xyz_pose_topics_.size());
  for (size_t i = 0; i < conv_xyz_pose_topics_.size(); i++) {
    size_t index = idx;
    xyz_to_gps_subs_pose_.push_back(
      dua_create_subscription<PoseStamped>(
        conv_xyz_pose_topics_.at(i),
        [this, index](const PoseStamped::SharedPtr msg) -> void {
          xyz_to_gps_clbk(index, msg);
        },
        dua_qos::Reliable::get_datum_qos(),
        xyz_to_gps_cgroups_pose_.at(i)));
    idx++;
  }

  xyz_to_gps_subs_posecov_.reserve(conv_xyz_posecov_topics_.size());
  for (size_t i = 0; i < conv_xyz_posecov_topics_.size(); i++) {
    size_t index = idx;
    xyz_to_gps_subs_posecov_.push_back(
      dua_create_subscription<PoseWithCovarianceStamped>(
        conv_xyz_posecov_topics_.at(i),
        [this, index](const PoseWithCovarianceStamped::SharedPtr msg) -> void {
          xyz_to_gps_clbk(index, msg);
        },
        dua_qos::Reliable::get_datum_qos(),
        xyz_to_gps_cgroups_posecov_.at(i)));
    idx++;
  }
}


void NavSatConverter::init_publishers()
{
  gps_to_xyz_pubs_odom_.reserve(conv_gps_fix_topics_.size());
  gps_to_xyz_pubs_point_.reserve(conv_gps_fix_topics_.size());
  gps_to_xyz_pubs_pose_.reserve(conv_gps_fix_topics_.size());
  gps_to_xyz_pubs_posecov_.reserve(conv_gps_fix_topics_.size());
  for (size_t i = 0; i < conv_gps_fix_topics_.size(); i++) {
    gps_to_xyz_pubs_odom_.push_back(
      dua_create_publisher<Odometry>(
        conv_gps_fix_topics_.at(i) + "/odometry",
        dua_qos::Reliable::get_datum_qos()));

    gps_to_xyz_pubs_point_.push_back(
      dua_create_publisher<PointStamped>(
        conv_gps_fix_topics_.at(i) + "/point",
        dua_qos::Reliable::get_datum_qos()));

    gps_to_xyz_pubs_pose_.push_back(
      dua_create_publisher<PoseStamped>(
        conv_gps_fix_topics_.at(i) + "/pose",
        dua_qos::Reliable::get_datum_qos()));

    gps_to_xyz_pubs_posecov_.push_back(
      dua_create_publisher<PoseWithCovarianceStamped>(
        conv_gps_fix_topics_.at(i) + "/pose_cov",
        dua_qos::Reliable::get_datum_qos()));
  }

  xyz_to_gps_pubs_fix_.reserve(
    conv_xyz_odometry_topics_.size() +
    conv_xyz_point_topics_.size() +
    conv_xyz_pose_topics_.size() +
    conv_xyz_posecov_topics_.size());
  for (size_t i = 0; i < conv_xyz_odometry_topics_.size(); i++) {
    xyz_to_gps_pubs_fix_.push_back(
      dua_create_publisher<NavSatFix>(
        conv_xyz_odometry_topics_.at(i) + "/fix",
        dua_qos::Reliable::get_datum_qos()));
  }
  for (size_t i = 0; i < conv_xyz_point_topics_.size(); i++) {
    xyz_to_gps_pubs_fix_.push_back(
      dua_create_publisher<NavSatFix>(
        conv_xyz_point_topics_.at(i) + "/fix",
        dua_qos::Reliable::get_datum_qos()));
  }
  for (size_t i = 0; i < conv_xyz_pose_topics_.size(); i++) {
    xyz_to_gps_pubs_fix_.push_back(
      dua_create_publisher<NavSatFix>(
        conv_xyz_pose_topics_.at(i) + "/fix",
        dua_qos::Reliable::get_datum_qos()));
  }
  for (size_t i = 0; i < conv_xyz_posecov_topics_.size(); i++) {
    xyz_to_gps_pubs_fix_.push_back(
      dua_create_publisher<NavSatFix>(
        conv_xyz_posecov_topics_.at(i) + "/fix",
        dua_qos::Reliable::get_datum_qos()));
  }
}


void NavSatConverter::init_service_servers()
{
  convert_gps_srv_ = dua_create_service_server<ConvertGPS>(
    convert_gps_srv_name_,
    std::bind(
      &NavSatConverter::convert_gps_clbk,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    convert_gps_cgroup_);

  convert_xyz_srv_ = dua_create_service_server<ConvertXYZ>(
    convert_xyz_srv_name_,
    std::bind(
      &NavSatConverter::convert_xyz_clbk,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    convert_xyz_cgroup_);

  update_earth_srv_ = dua_create_service_server<UpdateEarth>(
    update_earth_srv_name_,
    std::bind(
      &NavSatConverter::update_earth_clbk,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    update_earth_cgroup_);
}


void NavSatConverter::init_service_clients()
{
  get_transform_cli_ = dua_create_service_client<GetTransform>(
    get_transform_cli_name_);
}


void NavSatConverter::init_internals()
{
  coords = LocalCartesian(earth_gps_lat_, earth_gps_lon_, earth_gps_alt_);
}


} // namespace navsat_converter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(navsat_converter::NavSatConverter)
