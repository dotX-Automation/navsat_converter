/**
 * NavSat Converter subscriptions ruotine.
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


void NavSatConverter::coords_clbk(const NavSatFix::SharedPtr msg)
{
  if (msg->header.frame_id == earth_frame_) {
    std::lock_guard guard(coords_mutex);
    coords = LocalCartesian(msg->latitude, msg->longitude, msg->altitude);
  } else {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
      "NavSatFix message frame_id (%s) != Earth frame (%s)",
      msg->header.frame_id.c_str(), earth_frame_.c_str());
  }
}


void NavSatConverter::gps_to_xyz_clbk(size_t index, const NavSatFix::SharedPtr msg)
{
  Vector3d gps_pos, xyz_pos;
  Matrix3d gps_cov, xyz_cov;
  Isometry3d isometry;

  Header hdr;
  hdr.frame_id = target_frame_;
  hdr.stamp = msg->header.stamp;

  if (get_isometry(hdr, isometry)) {
    gps_pos.x() = msg->latitude;
    gps_pos.y() = msg->longitude;
    gps_pos.z() = msg->altitude;
    gps_cov(0, 0) = msg->position_covariance[0];
    gps_cov(0, 1) = msg->position_covariance[1];
    gps_cov(0, 2) = msg->position_covariance[2];
    gps_cov(1, 0) = msg->position_covariance[3];
    gps_cov(1, 1) = msg->position_covariance[4];
    gps_cov(1, 2) = msg->position_covariance[5];
    gps_cov(2, 0) = msg->position_covariance[6];
    gps_cov(2, 1) = msg->position_covariance[7];
    gps_cov(2, 2) = msg->position_covariance[8];
    gps_to_xyz(isometry, gps_pos, gps_cov, xyz_pos, xyz_cov);
    publish_gps_to_xyz(index, hdr, xyz_pos, xyz_cov);
  }
}


void NavSatConverter::xyz_to_gps_clbk(size_t index, const Odometry::SharedPtr msg)
{
  Vector3d xyz_pos, gps_pos;
  Matrix3d xyz_cov, gps_cov;
  Isometry3d isometry;

  if (get_isometry(msg->header, isometry)) {
    xyz_pos.x() = msg->pose.pose.position.x;
    xyz_pos.y() = msg->pose.pose.position.y;
    xyz_pos.z() = msg->pose.pose.position.z;
    xyz_cov(0, 0) = msg->pose.covariance[0];
    xyz_cov(0, 1) = msg->pose.covariance[1];
    xyz_cov(0, 2) = msg->pose.covariance[2];
    xyz_cov(1, 0) = msg->pose.covariance[6];
    xyz_cov(1, 1) = msg->pose.covariance[7];
    xyz_cov(1, 2) = msg->pose.covariance[8];
    xyz_cov(2, 0) = msg->pose.covariance[12];
    xyz_cov(2, 1) = msg->pose.covariance[13];
    xyz_cov(2, 2) = msg->pose.covariance[14];
    xyz_to_gps(isometry, xyz_pos, xyz_cov, gps_pos, gps_cov);
    publish_xyz_to_gps(index, msg->header, gps_pos, gps_cov);
  }
}


void NavSatConverter::xyz_to_gps_clbk(size_t index, const PointStamped::SharedPtr msg)
{
  Vector3d xyz_pos, gps_pos;
  Matrix3d xyz_cov, gps_cov;
  Isometry3d isometry;

  if (get_isometry(msg->header, isometry)) {
    xyz_pos.x() = msg->point.x;
    xyz_pos.y() = msg->point.y;
    xyz_pos.z() = msg->point.z;
    xyz_cov(0, 0) = 0.0;
    xyz_cov(0, 1) = 0.0;
    xyz_cov(0, 2) = 0.0;
    xyz_cov(1, 0) = 0.0;
    xyz_cov(1, 1) = 0.0;
    xyz_cov(1, 2) = 0.0;
    xyz_cov(2, 0) = 0.0;
    xyz_cov(2, 1) = 0.0;
    xyz_cov(2, 2) = 0.0;
    xyz_to_gps(isometry, xyz_pos, xyz_cov, gps_pos, gps_cov);
    publish_xyz_to_gps(index, msg->header, gps_pos, gps_cov);
  }
}


void NavSatConverter::xyz_to_gps_clbk(size_t index, const PoseStamped::SharedPtr msg)
{
  Vector3d xyz_pos, gps_pos;
  Matrix3d xyz_cov, gps_cov;
  Isometry3d isometry;

  if (get_isometry(msg->header, isometry)) {
    xyz_pos.x() = msg->pose.position.x;
    xyz_pos.y() = msg->pose.position.y;
    xyz_pos.z() = msg->pose.position.z;
    xyz_cov(0, 0) = 0.0;
    xyz_cov(0, 1) = 0.0;
    xyz_cov(0, 2) = 0.0;
    xyz_cov(1, 0) = 0.0;
    xyz_cov(1, 1) = 0.0;
    xyz_cov(1, 2) = 0.0;
    xyz_cov(2, 0) = 0.0;
    xyz_cov(2, 1) = 0.0;
    xyz_cov(2, 2) = 0.0;
    xyz_to_gps(isometry, xyz_pos, xyz_cov, gps_pos, gps_cov);
    publish_xyz_to_gps(index, msg->header, gps_pos, gps_cov);
  }
}


void NavSatConverter::xyz_to_gps_clbk(size_t index, const PoseWithCovarianceStamped::SharedPtr msg)
{
  Vector3d xyz_pos, gps_pos;
  Matrix3d xyz_cov, gps_cov;
  Isometry3d isometry;

  if (get_isometry(msg->header, isometry)) {
    xyz_pos.x() = msg->pose.pose.position.x;
    xyz_pos.y() = msg->pose.pose.position.y;
    xyz_pos.z() = msg->pose.pose.position.z;
    xyz_cov(0, 0) = msg->pose.covariance[0];
    xyz_cov(0, 1) = msg->pose.covariance[1];
    xyz_cov(0, 2) = msg->pose.covariance[2];
    xyz_cov(1, 0) = msg->pose.covariance[6];
    xyz_cov(1, 1) = msg->pose.covariance[7];
    xyz_cov(1, 2) = msg->pose.covariance[8];
    xyz_cov(2, 0) = msg->pose.covariance[12];
    xyz_cov(2, 1) = msg->pose.covariance[13];
    xyz_cov(2, 2) = msg->pose.covariance[14];
    xyz_to_gps(isometry, xyz_pos, xyz_cov, gps_pos, gps_cov);
    publish_xyz_to_gps(index, msg->header, gps_pos, gps_cov);
  }
}

} // NavSatConverter
