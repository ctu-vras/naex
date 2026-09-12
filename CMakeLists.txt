cmake_minimum_required(VERSION 3.16)
project(naex)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- dependencies ---------------------------------------------------------
# Keep every find_package call in this block so that new targets only have to
# append a target_link_libraries() below.
find_package(ament_cmake REQUIRED)

find_package(Boost REQUIRED CONFIG COMPONENTS graph)
find_package(Eigen3 REQUIRED NO_MODULE)

find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# ClearEntireCostmap: clear_plan_map keeps the nav2 name/type the robots use.
find_package(nav2_msgs REQUIRED)
# compute_path_to_pose action server (nav2_util::SimpleActionServer) and its
# result-code exceptions (nav2_core::Planner*Error etc.).
find_package(nav2_util REQUIRED)
find_package(nav2_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

# FLANN ships neither a CMake config package nor a .pc file on Ubuntu 24.04,
# so it is located by hand.  The parts used here (flann::Index, Matrix,
# KDTreeSingleIndexParams, SearchParams) are header-only; the only library
# actually needed at link time is liblz4, pulled in by
# flann/util/serialization.h.  The imported target keeps the include directory
# SYSTEM, so FLANN's own headers do not trip -Wall -Wextra -Wpedantic.
find_path(FLANN_INCLUDE_DIR NAMES flann/flann.hpp)
find_library(LZ4_LIBRARY NAMES lz4)
if(NOT FLANN_INCLUDE_DIR OR NOT LZ4_LIBRARY)
    message(FATAL_ERROR
        "FLANN (libflann-dev) and LZ4 (liblz4-dev) are required by naex")
endif()
add_library(naex::flann INTERFACE IMPORTED)
set_target_properties(naex::flann PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${FLANN_INCLUDE_DIR}"
  INTERFACE_LINK_LIBRARIES "${LZ4_LIBRARY}")

# --- targets --------------------------------------------------------------
# Non-template code shared by the executables and the component.  Everything
# here compiles exactly once; the headers keep only templates and the small
# accessors that have to stay inline.
add_library(naex_core STATIC
  src/clouds.cpp
  src/spherical_projection.cpp
  src/traversability.cpp
  src/map/map_cloud.cpp
  src/map/map_costs.cpp
  src/map/map_index.cpp
  src/map/map_occupancy.cpp)
set_target_properties(naex_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(naex_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(naex_core
  PUBLIC
    Eigen3::Eigen
    rclcpp::rclcpp
    tf2::tf2
    tf2_eigen::tf2_eigen
    ${geometry_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
  PRIVATE
    naex::flann)

# Grid planner: the Planner member bodies and the path helpers.  Split out of
# the headers so that touching a body recompiles one translation unit instead
# of every node that includes grid/planner.h.  Separate from naex_core because
# it needs the nav_msgs/tf2_ros/visualization_msgs stack that naex_core does
# not.
add_library(naex_grid STATIC
  src/grid/path.cpp
  src/grid/planner.cpp)
set_target_properties(naex_grid PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(naex_grid PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(naex_grid
  PUBLIC
    Boost::graph
    Eigen3::Eigen
    rclcpp::rclcpp
    tf2::tf2
    tf2_ros::tf2_ros
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${nav2_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${std_srvs_TARGETS}
    ${visualization_msgs_TARGETS}
    nav2_util::nav2_util_core
    nav2_core::nav2_core
  PRIVATE
    naex_core
    tf2_eigen::tf2_eigen
    tf2_geometry_msgs::tf2_geometry_msgs)

add_executable(grid_planner src/grid_planner_node.cpp)
target_include_directories(grid_planner PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(grid_planner PRIVATE
  naex_core
  naex_grid
  Boost::graph
  Eigen3::Eigen
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_eigen::tf2_eigen
  tf2_geometry_msgs::tf2_geometry_msgs
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${visualization_msgs_TARGETS})

# Local re-planner around the path published by grid_planner; plans on a
# single-scan grid in the sensor frame (grid/mule_planner.h).
add_executable(mule_planner src/mule_planner_node.cpp)
target_include_directories(mule_planner PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(mule_planner PRIVATE
  naex_core
  naex_grid
  Boost::graph
  Eigen3::Eigen
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_eigen::tf2_eigen
  tf2_geometry_msgs::tf2_geometry_msgs
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${visualization_msgs_TARGETS})

# --- additional targets are appended below by the port ---

# Point-map planner (FLANN-based point map, Boost.Graph Dijkstra).
add_executable(planner
  src/planner_node.cpp
  src/planner.cpp)
target_include_directories(planner PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(planner PRIVATE
  naex_core
  naex::flann
  Boost::graph
  Eigen3::Eigen
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_eigen::tf2_eigen
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS})

# Traversability: a composable component; the standalone executable
# traversability_node is generated from it below.
add_library(traversability_component SHARED src/traversability_node.cpp)
target_include_directories(traversability_component PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(traversability_component
  PRIVATE
    naex_core
    naex::flann
  PUBLIC
    Eigen3::Eigen
    rclcpp::rclcpp
    rclcpp_components::component
    tf2::tf2
    tf2_ros::tf2_ros
    tf2_eigen::tf2_eigen
    ${geometry_msgs_TARGETS}
    ${sensor_msgs_TARGETS})
rclcpp_components_register_node(traversability_component
  PLUGIN "naex::TraversabilityNode"
  EXECUTABLE traversability_node)

# Spherical projection model estimation; needs neither FLANN nor LZ4.
add_executable(lidar_model src/lidar_model.cpp)
target_include_directories(lidar_model PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(lidar_model PRIVATE
  naex_core
  Eigen3::Eigen
  rclcpp::rclcpp
  tf2::tf2
  tf2_eigen::tf2_eigen
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS})

# --- install --------------------------------------------------------------
install(TARGETS
    grid_planner
    lidar_model
    mule_planner
    planner
  DESTINATION lib/${PROJECT_NAME})

# The component library goes to lib/ (not lib/${PROJECT_NAME}) so that
# component_container can dlopen it; the executable generated by
# rclcpp_components_register_node is installed by that macro itself.
install(TARGETS naex_core naex_grid traversability_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_libraries(traversability_component)
ament_export_dependencies(
  Eigen3
  geometry_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs)

install(PROGRAMS
    scripts/bench_grid_planner.py
    scripts/get_plan.py
    scripts/mock_map_publisher.py
    scripts/mock_tf_publisher.py
    scripts/path_follower.py
    scripts/republish.py
    scripts/test_planner_client.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  PATTERN "__pycache__" EXCLUDE)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

# --- tests ----------------------------------------------------------------
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # clang-format (.clang-format) is the formatting authority here; uncrustify
    # and cpplint disagree with it, so skip both instead of reformatting.
    set(ament_cmake_uncrustify_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    # The ROS 1 tree still being ported trips these; re-enable once it is gone.
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_flake8_FOUND TRUE)
    set(ament_cmake_pep257_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()

    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_grid test/test_grid.cpp)
    if(TARGET test_grid)
        target_include_directories(test_grid PRIVATE
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
        target_link_libraries(test_grid
            naex_grid
            Boost::graph
            Eigen3::Eigen
            rclcpp::rclcpp
            ${nav_msgs_TARGETS}
            ${sensor_msgs_TARGETS}
            ${visualization_msgs_TARGETS})
    endif()
endif()

ament_package()
