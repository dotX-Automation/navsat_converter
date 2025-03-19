/**
 * NavSat Converter publishers routine.
 *
 * Giorgio Manca <giorgio.manca.97@gmail.com>
 *
 * December 6, 2024
 */

/**
 * Copyright 2024 dotX Automation s.r.l.
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


void NavSatConverter::publish_gps_to_xyz(
  size_t index, const Header & hdr, const Vector3d & xyz,
  const Matrix3d & cov)
{
  std::array<double, 36UL> covariance;
  covariance[0] = cov(0, 0);
  covariance[1] = cov(0, 1);
  covariance[2] = cov(0, 2);
  covariance[6] = cov(1, 0);
  covariance[7] = cov(1, 1);
  covariance[8] = cov(1, 2);
  covariance[12] = cov(2, 0);
  covariance[13] = cov(2, 1);
  covariance[14] = cov(2, 2);

  Odometry odom;
  PointStamped point;
  PoseStamped pose;
  PoseWithCovarianceStamped posecov;

  odom.header = hdr;
  odom.pose.pose.position.x = xyz.x();
  odom.pose.pose.position.y = xyz.y();
  odom.pose.pose.position.z = xyz.z();
  odom.pose.covariance = covariance;

  point.header = hdr;
  point.point.x = xyz.x();
  point.point.y = xyz.y();
  point.point.z = xyz.z();

  pose.header = hdr;
  pose.pose.position.x = xyz.x();
  pose.pose.position.y = xyz.y();
  pose.pose.position.z = xyz.z();

  posecov.header = hdr;
  posecov.pose.pose.position.x = xyz.x();
  posecov.pose.pose.position.y = xyz.y();
  posecov.pose.pose.position.z = xyz.z();
  posecov.pose.covariance = covariance;

  gps_to_xyz_pubs_odom_.at(index)->publish(odom);
  gps_to_xyz_pubs_point_.at(index)->publish(point);
  gps_to_xyz_pubs_pose_.at(index)->publish(pose);
  gps_to_xyz_pubs_posecov_.at(index)->publish(posecov);
}


void NavSatConverter::publish_xyz_to_gps(
  size_t index, const Header & hdr, const Vector3d & gps,
  const Matrix3d & cov)
{
  std::array<double, 9UL> covariance;
  covariance[0] = cov(0, 0);
  covariance[1] = cov(0, 1);
  covariance[2] = cov(0, 2);
  covariance[3] = cov(1, 0);
  covariance[4] = cov(1, 1);
  covariance[5] = cov(1, 2);
  covariance[6] = cov(2, 0);
  covariance[7] = cov(2, 1);
  covariance[8] = cov(2, 2);

  NavSatFix nsf;

  nsf.header = hdr;
  nsf.latitude = gps.x();
  nsf.longitude = gps.y();
  nsf.altitude = gps.z();
  nsf.position_covariance = covariance;
  nsf.position_covariance_type = NavSatFix::COVARIANCE_TYPE_APPROXIMATED;

  xyz_to_gps_pubs_fix_.at(index)->publish(nsf);
}

} // NavSatConverter
