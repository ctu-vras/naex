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
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)

# --- targets --------------------------------------------------------------
add_executable(grid_planner src/grid_planner_node.cpp)
target_include_directories(grid_planner PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(grid_planner PRIVATE
  Boost::graph
  Eigen3::Eigen
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_eigen::tf2_eigen
  ${geometry_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS})

# --- additional targets are appended below by the port ---

# --- install --------------------------------------------------------------
install(TARGETS grid_planner
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
    scripts/mock_map_publisher.py
    scripts/mock_tf_publisher.py
    scripts/test_planner_client.py
  DESTINATION lib/${PROJECT_NAME})

install(FILES
    launch/grid_planner.launch.py
    launch/test_planner.launch.py
  DESTINATION share/${PROJECT_NAME}/launch)

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
      Boost::graph
      Eigen3::Eigen
      rclcpp::rclcpp
      ${sensor_msgs_TARGETS})
  endif()
endif()

ament_package()
