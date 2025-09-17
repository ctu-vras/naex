cmake_minimum_required(VERSION 3.9)
project(naex)

find_package(ament_cmake REQUIRED)

find_package(Boost COMPONENTS graph chrono REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(OpenMP REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(eigen REQUIRED eigen3)
include_directories(${eigen_INCLUDE_DIRS})
link_directories(${eigen_LIBRARY_DIRS})

find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_executable(grid_planner src/grid_planner_node.cpp)
ament_target_dependencies(
    grid_planner
        Boost
        rclcpp
        tf2_ros
        nav_msgs
        std_msgs
        nav2_msgs
        tf2_eigen
        sensor_msgs
        geometry_msgs
)
target_link_libraries(
    grid_planner
        Boost::graph
        Boost::chrono
        Eigen3::Eigen
        OpenMP::OpenMP_CXX
)

install(
    TARGETS
        grid_planner
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY
        launch
        config
    DESTINATION share/${PROJECT_NAME}
)

install(
    PROGRAMS
        scripts/mock_map_publisher.py
        scripts/mock_tf_publisher.py
        scripts/test_planner_client.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_package()
