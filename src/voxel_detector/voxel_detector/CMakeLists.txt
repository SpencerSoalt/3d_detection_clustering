cmake_minimum_required(VERSION 3.8)
project(voxel_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Voxel detector
add_executable(voxel_detector_node src/voxel_detector_node.cpp)

ament_target_dependencies(voxel_detector_node
  rclcpp
  sensor_msgs
  visualization_msgs
  pcl_conversions
)

target_link_libraries(voxel_detector_node
  ${PCL_LIBRARIES}
)

# Euclidean detector
add_executable(euclidean_detector_node src/euclidean_detector_node.cpp)

ament_target_dependencies(euclidean_detector_node
  rclcpp
  sensor_msgs
  visualization_msgs
  pcl_conversions
)

target_link_libraries(euclidean_detector_node
  ${PCL_LIBRARIES}
)

# DBSCAN detector
add_executable(dbscan_detector_node src/dbscan_detector_node.cpp)

ament_target_dependencies(dbscan_detector_node
  rclcpp
  sensor_msgs
  visualization_msgs
  pcl_conversions
)

target_link_libraries(dbscan_detector_node
  ${PCL_LIBRARIES}
)

# Install all executables
install(TARGETS
  voxel_detector_node
  euclidean_detector_node
  dbscan_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()