/**
 * NavSat Converter node services.
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

void NavSatConverter::convert_gps_clbk(
  const ConvertGPS::Request::SharedPtr req,
  const ConvertGPS::Response::SharedPtr res)
{
  Vector3d gps_pos, xyz_pos;
  Matrix3d gps_cov, xyz_cov;
  Isometry3d isometry;

  if(req->header.frame_id != earth_frame_) {
    res->success = false;
    RCLCPP_ERROR(this->get_logger(),
      "GPS conversion failed: GPS coordinates source frame_id is not equal to Earth frame name");
    return;
  }

  Header hdr;
  hdr.stamp = req->header.stamp;
  hdr.frame_id = req->target_frame_id;

  if (get_isometry(hdr, isometry)) {
    gps_pos.x() = req->lat;
    gps_pos.y() = req->lon;
    gps_pos.z() = req->alt;
    gps_cov(0, 0) = req->covariance[0];
    gps_cov(0, 1) = req->covariance[1];
    gps_cov(0, 2) = req->covariance[2];
    gps_cov(1, 0) = req->covariance[3];
    gps_cov(1, 1) = req->covariance[4];
    gps_cov(1, 2) = req->covariance[5];
    gps_cov(2, 0) = req->covariance[6];
    gps_cov(2, 1) = req->covariance[7];
    gps_cov(2, 2) = req->covariance[8];
    gps_to_xyz(isometry, gps_pos, gps_cov, xyz_pos, xyz_cov);
    res->success = true;
    res->header.frame_id = req->target_frame_id;
    res->header.stamp = req->header.stamp;
    res->x = xyz_pos.x();
    res->y = xyz_pos.y();
    res->z = xyz_pos.z();
    res->covariance[0] = xyz_cov(0, 0);
    res->covariance[1] = xyz_cov(0, 1);
    res->covariance[2] = xyz_cov(0, 2);
    res->covariance[3] = xyz_cov(1, 0);
    res->covariance[4] = xyz_cov(1, 1);
    res->covariance[5] = xyz_cov(1, 2);
    res->covariance[6] = xyz_cov(2, 0);
    res->covariance[7] = xyz_cov(2, 1);
    res->covariance[8] = xyz_cov(2, 2);
  } else {
    res->success = false;
    RCLCPP_ERROR(this->get_logger(),
      "GPS conversion failed: XYZ coordinates target frame_id is not valid");
  }
}

void NavSatConverter::convert_xyz_clbk(
  const ConvertXYZ::Request::SharedPtr req,
  const ConvertXYZ::Response::SharedPtr res)
{
  Vector3d xyz_pos, gps_pos;
  Matrix3d xyz_cov, gps_cov;
  Isometry3d isometry;

  if(req->target_frame_id != earth_frame_) {
    res->success = false;
    RCLCPP_ERROR(this->get_logger(),
      "XYZ conversion failed: GPS coordinates target frame_id is not equal to Earth frame name");
    return;
  }

  Header hdr;
  hdr.stamp = req->header.stamp;
  hdr.frame_id = req->header.frame_id;

  if (get_isometry(hdr, isometry)) {
    xyz_pos.x() = req->x;
    xyz_pos.y() = req->y;
    xyz_pos.z() = req->z;
    xyz_cov(0, 0) = req->covariance[0];
    xyz_cov(0, 1) = req->covariance[1];
    xyz_cov(0, 2) = req->covariance[2];
    xyz_cov(1, 0) = req->covariance[3];
    xyz_cov(1, 1) = req->covariance[4];
    xyz_cov(1, 2) = req->covariance[5];
    xyz_cov(2, 0) = req->covariance[6];
    xyz_cov(2, 1) = req->covariance[7];
    xyz_cov(2, 2) = req->covariance[8];
    xyz_to_gps(isometry, xyz_pos, xyz_cov, gps_pos, gps_cov);
    res->success = true;
    res->header.frame_id = earth_frame_;
    res->header.stamp = req->header.stamp;
    res->lat = gps_pos.x();
    res->lon = gps_pos.y();
    res->alt = gps_pos.z();
    res->covariance[0] = gps_cov(0, 0);
    res->covariance[1] = gps_cov(0, 1);
    res->covariance[2] = gps_cov(0, 2);
    res->covariance[3] = gps_cov(1, 0);
    res->covariance[4] = gps_cov(1, 1);
    res->covariance[5] = gps_cov(1, 2);
    res->covariance[6] = gps_cov(2, 0);
    res->covariance[7] = gps_cov(2, 1);
    res->covariance[8] = gps_cov(2, 2);
  } else {
    res->success = false;
    RCLCPP_ERROR(this->get_logger(),
      "XYZ conversion failed: XYZ coordinates source frame_id is not valid");
  }
}

void NavSatConverter::update_earth_clbk(
  const UpdateEarth::Request::SharedPtr req,
  const UpdateEarth::Response::SharedPtr res)
{
  if (req->header.frame_id == earth_frame_) {
    {
      std::lock_guard guard(coords_mutex);
      coords = LocalCartesian(req->lat, req->lon, req->alt);
    }
    res->success = true;
  } else {
    res->success = false;
    RCLCPP_ERROR(this->get_logger(),
      "Earth coordinates update failed: frame_id is not the same");
  }
}

bool NavSatConverter::get_isometry(const Header & hdr, Isometry3d & isometry)
{
  auto req = std::make_shared<GetTransform::Request>();
  req->source.frame_id = earth_frame_;
  req->target.frame_id = hdr.frame_id;
  req->source.stamp = tf_ignore_stamp_ ? rclcpp::Time() : rclcpp::Time(hdr.stamp);
  req->target.stamp = req->source.stamp;
  req->timeout = rclcpp::Duration(std::chrono::nanoseconds(1000 * tf_timeout_ms_));

  auto resp = get_transform_cli_->call_sync(req);
  if (resp->result.result == CommandResultStamped::ERROR) {
    RCLCPP_ERROR(this->get_logger(),
      "Error requesting transform from %s to %s",
      earth_frame_.c_str(), hdr.frame_id.c_str());
    return false;
  }

  resp->transform.transform.translation;
  resp->transform.transform.rotation;

  Vector3d vect = Vector3d(
    resp->transform.transform.translation.x,
    resp->transform.transform.translation.y,
    resp->transform.transform.translation.z);

  Quaterniond quat = Quaterniond(
    resp->transform.transform.rotation.w,
    resp->transform.transform.rotation.x,
    resp->transform.transform.rotation.y,
    resp->transform.transform.rotation.z);

  isometry.translation() = vect;
  isometry.linear() = quat.toRotationMatrix();

  return true;
}

} // namespace navsat_converter
