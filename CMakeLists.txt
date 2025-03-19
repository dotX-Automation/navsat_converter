cmake_minimum_required(VERSION 3.10)
project(navsat_converter)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(dua_app_management REQUIRED)
find_package(dua_common_interfaces REQUIRED)
find_package(dua_geometry_interfaces REQUIRED)
find_package(dua_node_cpp REQUIRED)
find_package(dua_qos_cpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 3.4 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(params_manager_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(simple_serviceclient_cpp REQUIRED)
find_package(std_msgs REQUIRED)

# Geographiclib installs FindGeographicLib.cmake to this non-standard location
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)

# Generate parameters source code
generate_init_parameters(
  YAML_FILE "${CMAKE_CURRENT_SOURCE_DIR}/src/navsat_converter/params.yaml"
  OUT_FILE "init_parameters.cpp")

# Configure NavSat Converter node source files
file(GLOB NAVSAT_CONVERTER_SRCS
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  "src/navsat_converter/*.cpp")
list(APPEND NAVSAT_CONVERTER_SRCS init_parameters.cpp)

# NavSat Converter
add_library(navsat_converter SHARED ${NAVSAT_CONVERTER_SRCS})
target_compile_definitions(navsat_converter PRIVATE COMPOSITION_BUILDING_DLL)
target_include_directories(navsat_converter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(navsat_converter
  Eigen3::Eigen
  ${GeographicLib_LIBRARIES})
target_compile_features(navsat_converter PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(navsat_converter
  dua_common_interfaces
  dua_geometry_interfaces
  dua_node_cpp
  dua_qos_cpp
  geometry_msgs
  nav_msgs
  params_manager_cpp
  rclcpp
  rclcpp_components
  sensor_msgs
  simple_serviceclient_cpp
  std_msgs)
rclcpp_components_register_nodes(navsat_converter "navsat_converter::NavSatConverter")

# NavSat Converter standalone application
add_executable(navsat_converter_app
  src/navsat_converter_app.cpp)
target_include_directories(navsat_converter_app PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(navsat_converter_app PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(navsat_converter_app navsat_converter)
ament_target_dependencies(navsat_converter_app
  dua_app_management
  rclcpp)

# Install NavSat Converter component
install(TARGETS navsat_converter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install NavSat Converter app
install(TARGETS navsat_converter_app
  DESTINATION lib/${PROJECT_NAME})

# Install Config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

# Install Launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
