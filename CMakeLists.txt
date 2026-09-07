cmake_minimum_required(VERSION 3.10)
project(modular_3d_localizer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(FAST_GICP_PREFIX "" CACHE PATH "Prefix where fast_gicp is installed")

if(FAST_GICP_PREFIX)
  list(PREPEND CMAKE_PREFIX_PATH "${FAST_GICP_PREFIX}")
endif()
find_package(fast_gicp REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(modular_3d_localizer_node
    src/modular_3d_localizer_node.cpp
    src/map_loader.cpp
    src/point_cloud_preprocessor.cpp
    src/scan_processing_controller.cpp
    src/local_map_extractor.cpp
    src/motion_model/motion_model_factory.cpp
    src/motion_model/odometry_motion_model.cpp
    src/registration_backend_factory.cpp
    src/localization_backend/gicp_registration.cpp
    src/localization_backend/coarse_to_fine_gicp_registration.cpp
    src/localization_backend/fast_gicp_registration.cpp
    src/main.cpp
  )
ament_target_dependencies(modular_3d_localizer_node
  rclcpp
  sensor_msgs
  geometry_msgs
  diagnostic_msgs
  nav_msgs
  tf2_ros
  pcl_conversions
)
target_include_directories(modular_3d_localizer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_include_directories(modular_3d_localizer_node PRIVATE
  ${fast_gicp_INCLUDE_DIRS})
target_link_libraries(modular_3d_localizer_node
  ${PCL_LIBRARIES}
  ${fast_gicp_LIBRARIES}
)
add_executable(initial_pose_publisher
  src/initial_pose/initial_pose_publisher.cpp
  src/initial_pose/initial_pose_publisher_main.cpp
)
add_executable(tum_trajectory_evaluator
  src/evaluation/tum_trajectory_evaluator.cpp
  src/evaluation/tum_trajectory_evaluator_main.cpp
)
ament_target_dependencies(tum_trajectory_evaluator
  rclcpp
  geometry_msgs
)
target_include_directories(tum_trajectory_evaluator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(tum_trajectory_evaluator Eigen3::Eigen)
ament_target_dependencies(initial_pose_publisher
  rclcpp
  geometry_msgs
  tf2
)
target_include_directories(initial_pose_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
install(TARGETS
  modular_3d_localizer_node
  initial_pose_publisher
  tum_trajectory_evaluator
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch rviz config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
