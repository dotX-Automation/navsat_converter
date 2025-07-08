/**
 * NavSat Converter utilities.
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


void NavSatConverter::gps_to_xyz(
  const Eigen::Isometry3d & isometry,
  const Vector3d & gps_pos, const Matrix3d & gps_cov,
  Vector3d & xyz_pos, Matrix3d & xyz_cov)
{
  {
    std::lock_guard guard(coords_mutex);
    coords.Forward(gps_pos.x(), gps_pos.y(), gps_pos.z(), xyz_pos.x(), xyz_pos.y(), xyz_pos.z());
  }
  xyz_pos = isometry.inverse() * xyz_pos;
  xyz_cov = isometry.linear().transpose() * gps_cov * isometry.linear();
}


void NavSatConverter::xyz_to_gps(
  const Eigen::Isometry3d & isometry,
  const Vector3d & xyz_pos, const Matrix3d & xyz_cov,
  Vector3d & gps_pos, Matrix3d & gps_cov)
{
  Vector3d tmp_pos = isometry * xyz_pos;
  {
    std::lock_guard guard(coords_mutex);
    coords.Reverse(tmp_pos.x(), tmp_pos.y(), tmp_pos.z(), gps_pos.x(), gps_pos.y(), gps_pos.z());
  }
  gps_cov = isometry.linear() * xyz_cov * isometry.linear().transpose();
}

} // namespace navsat_converter
