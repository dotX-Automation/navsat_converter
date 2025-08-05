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

#pragma once

#include <bitset>

#include <fcntl.h>
#include <pthread.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <mutex>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>

#include <dua_node_cpp/dua_node.hpp>
#include <dua_qos_cpp/dua_qos.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/unsupported/Eigen/EulerAngles>

#include <GeographicLib/LocalCartesian.hpp>

#include <dua_common_interfaces/msg/command_result_stamped.hpp>
#include <dua_geometry_interfaces/srv/convert_gps.hpp>
#include <dua_geometry_interfaces/srv/convert_xyz.hpp>
#include <dua_geometry_interfaces/srv/get_transform.hpp>
#include <dua_geometry_interfaces/srv/update_earth.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/header.hpp>

using namespace dua_common_interfaces::msg;
using namespace dua_geometry_interfaces::srv;
using namespace Eigen;
using namespace GeographicLib;
using namespace geometry_msgs::msg;
using namespace nav_msgs::msg;
using namespace rcl_interfaces::msg;
using namespace sensor_msgs::msg;
using namespace std_msgs::msg;

namespace navsat_converter
{

class NavSatConverter : public dua_node::NodeBase
{
public:
  /**
   * @brief Builds a new NavSatConverter node.
   *
   * @param opts Node options.
   *
   * @throws RuntimeError
   */
  NavSatConverter(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions());

  /**
   * @brief Finalizes node operation.
   */
  ~NavSatConverter();

private:
  /* Node Initialization Routines. */
  void init_parameters() override;
  void init_cgroups() override;
  void init_subscribers() override;
  void init_publishers() override;
  void init_service_servers() override;
  void init_service_clients() override;

  /**
   * @brief Routine to initialize node structures.
   */
  void init_internals();

  /* Node Parameters. */
  std::vector<std::string> conv_gps_fix_topics_ = {};
  std::vector<std::string> conv_xyz_odometry_topics_ = {};
  std::vector<std::string> conv_xyz_point_topics_ = {};
  std::vector<std::string> conv_xyz_pose_topics_ = {};
  std::vector<std::string> conv_xyz_posecov_topics_ = {};
  std::string earth_frame_ = "";
  double earth_gps_lat_ = 0.0;
  double earth_gps_lon_ = 0.0;
  double earth_gps_alt_ = 0.0;
  std::string target_frame_ = "";
  bool earth_topic_stream_ = false;
  bool tf_ignore_stamp_ = false;
  int64_t tf_timeout_ms_ = 0;

  /* Node Parameters Validators. */
  bool validate_conv_gps_fix_topics(const rclcpp::Parameter & p);
  bool validate_conv_xyz_odometry_topics(const rclcpp::Parameter & p);
  bool validate_conv_xyz_point_topics(const rclcpp::Parameter & p);
  bool validate_conv_xyz_pose_topics(const rclcpp::Parameter & p);
  bool validate_conv_xyz_posecov_topics(const rclcpp::Parameter & p);

  /* Node Variables. */
  LocalCartesian coords;
  std::mutex coords_mutex;

  /* Publishers. */
  std::vector<rclcpp::Publisher<Odometry>::SharedPtr> gps_to_xyz_pubs_odom_;
  std::vector<rclcpp::Publisher<PointStamped>::SharedPtr> gps_to_xyz_pubs_point_;
  std::vector<rclcpp::Publisher<PoseStamped>::SharedPtr> gps_to_xyz_pubs_pose_;
  std::vector<rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr> gps_to_xyz_pubs_posecov_;
  std::vector<rclcpp::Publisher<NavSatFix>::SharedPtr> xyz_to_gps_pubs_fix_;

  /* Publishers Routines. */

  /**
   * @brief Convert GPS coordinates to cartesian coordinates.
   *
   * @param index Conversion index.
   * @param xyz Cartesian coordinates to publish.
   */
  void publish_gps_to_xyz(
    size_t index, const Header & hdr, const Vector3d & xyz,
    const Matrix3d & cov);

  /**
   * @brief Convert cartesian coordinates to GPS coordinates.
   *
   * @param index Conversion index.
   * @param gps GPS coordinates to publish.
   */
  void publish_xyz_to_gps(
    size_t index, const Header & hdr, const Vector3d & gps,
    const Matrix3d & cov);

  /* Subscriptions. */
  rclcpp::Subscription<NavSatFix>::SharedPtr coords_sub_;
  std::vector<rclcpp::Subscription<NavSatFix>::SharedPtr> gps_to_xyz_subs_fix_;
  std::vector<rclcpp::Subscription<Odometry>::SharedPtr> xyz_to_gps_subs_odom_;
  std::vector<rclcpp::Subscription<PointStamped>::SharedPtr> xyz_to_gps_subs_point_;
  std::vector<rclcpp::Subscription<PoseStamped>::SharedPtr> xyz_to_gps_subs_pose_;
  std::vector<rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr> xyz_to_gps_subs_posecov_;

  /* Subscriptions Topics. */
  static const std::string coords_topic_;

  /* Subscriptions Callback Groups. */
  rclcpp::CallbackGroup::SharedPtr coords_cgroup_;
  std::vector<rclcpp::CallbackGroup::SharedPtr> gps_to_xyz_cgroups_fix_;
  std::vector<rclcpp::CallbackGroup::SharedPtr> xyz_to_gps_cgroups_odometry_;
  std::vector<rclcpp::CallbackGroup::SharedPtr> xyz_to_gps_cgroups_point_;
  std::vector<rclcpp::CallbackGroup::SharedPtr> xyz_to_gps_cgroups_pose_;
  std::vector<rclcpp::CallbackGroup::SharedPtr> xyz_to_gps_cgroups_posecov_;

  /* Subscriptions Callbacks. */

  /**
   * @brief Updates earth frame gps coordinates.
   *
   * @param msg NavSatFix message to parse.
   */
  void coords_clbk(const NavSatFix::SharedPtr msg);

  /**
   * @brief Receives NavSatFix messages to convert.
   *
   * @param index Conversion index.
   * @param msg NavSatFix message to parse.
   */
  void gps_to_xyz_clbk(size_t index, const NavSatFix::SharedPtr msg);

  /**
   * @brief Receives Odometry messages to convert.
   *
   * @param index Conversion index.
   * @param msg Odometry message to parse.
   */
  void xyz_to_gps_clbk(size_t index, const Odometry::SharedPtr msg);

  /**
   * @brief Receives PointStamped messages to convert.
   *
   * @param index Conversion index.
   * @param msg PointStamped message to parse.
   */
  void xyz_to_gps_clbk(size_t index, const PointStamped::SharedPtr msg);

  /**
   * @brief Receives PoseStamped messages to convert.
   *
   * @param index Conversion index.
   * @param msg PoseStamped message to parse.
   */
  void xyz_to_gps_clbk(size_t index, const PoseStamped::SharedPtr msg);

  /**
   * @brief Receives PoseWithCovarianceStamped messages to convert.
   *
   * @param index Conversion index.
   * @param msg PoseWithCovarianceStamped message to parse.
   */
  void xyz_to_gps_clbk(size_t index, const PoseWithCovarianceStamped::SharedPtr msg);

  /* Utility routines. */

  /**
   * @brief Routine to convert geographic coordinates in cartesian coordinates.
   *
   * @param isometry isometry from gps frame to cartesian frame
   * @param gps_pos input geographic coordinates
   * @param gps_cov input geographic covariance
   * @param xyz_pos output cartesian coordinates
   * @param xyz_cov output cartesian covariance
   */
  void gps_to_xyz(
    const Eigen::Isometry3d & isometry,
    const Vector3d & gps_pos, const Matrix3d & gps_cov,
    Vector3d & xyz_pos, Matrix3d & xyz_cov);

  /**
   * @brief Trasform cartesian coordinates in geographic coordinates.
   *
   * @param isometry isometry from gps frame to cartesian frame
   * @param xyz_pos input cartesian coordinates
   * @param xyz_cov input cartesian covariance
   * @param gps_pos output geographic coordinates
   * @param gps_cov output geographic covariance
   */
  void xyz_to_gps(
    const Eigen::Isometry3d & isometry,
    const Vector3d & xyz_pos, const Matrix3d & xyz_cov,
    Vector3d & gps_pos, Matrix3d & gps_cov);

  /* Service Servers. */
  rclcpp::Service<ConvertGPS>::SharedPtr convert_gps_srv_;
  rclcpp::Service<ConvertXYZ>::SharedPtr convert_xyz_srv_;
  rclcpp::Service<UpdateEarth>::SharedPtr update_earth_srv_;

  /* Service Servers Names. */
  static const std::string convert_gps_srv_name_;
  static const std::string convert_xyz_srv_name_;
  static const std::string update_earth_srv_name_;

  /* Service Servers Callback Groups. */
  rclcpp::CallbackGroup::SharedPtr convert_gps_cgroup_;
  rclcpp::CallbackGroup::SharedPtr convert_xyz_cgroup_;
  rclcpp::CallbackGroup::SharedPtr update_earth_cgroup_;

  /* Service Servers Callbacks. */

  /**
 * @brief Convert GPS coordinate service callback.
 *
 * @param req Service request.
 * @param res Service response.
 */
  void convert_gps_clbk(
    const ConvertGPS::Request::SharedPtr req,
    const ConvertGPS::Response::SharedPtr res);

  /**
   * @brief Convert XYZ coordinate service callback.
   *
   * @param req Service request.
   * @param res Service response.
   */
  void convert_xyz_clbk(
    const ConvertXYZ::Request::SharedPtr req,
    const ConvertXYZ::Response::SharedPtr res);

  /**
   * @brief Update earth frame coordinates service callback.
   *
   * @param req Service request.
   * @param res Service response.
   */
  void update_earth_clbk(
    const UpdateEarth::Request::SharedPtr req,
    const UpdateEarth::Response::SharedPtr res);

  /* Service Clients */
  simple_serviceclient::Client<GetTransform>::SharedPtr get_transform_cli_;

  /* Service Clients Names */
  static const std::string get_transform_cli_name_;

  /* Service Clients Routines */

  /**
   * @brief Routine to get isometry from earth frame to specified frame.
   *
   * @param hdr target frame
   * @param isometry output isometry
   */
  bool get_isometry(const Header & hdr, Isometry3d & isometry);
};

} // namespace navsat_converter
