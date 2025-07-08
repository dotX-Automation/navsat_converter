/**
 * NavSat Converter node constants.
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
/* Subscriptions Topics. */
const std::string NavSatConverter::coords_topic_ = "~/earth_coords";

/* Service Servers. */
const std::string NavSatConverter::convert_gps_srv_name_ = "~/convert_gps";
const std::string NavSatConverter::convert_xyz_srv_name_ = "~/convert_xyz";
const std::string NavSatConverter::update_earth_srv_name_ = "~/update_earth";

/* Service Client Names */
const std::string NavSatConverter::get_transform_cli_name_ = "/dua_tf_server/get_transform";

} // namespace navsat_converter
