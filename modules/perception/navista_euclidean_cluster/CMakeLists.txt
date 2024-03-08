cmake_minimum_required(VERSION 3.8)
project(navista_euclidean_cluster)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(sensor_msgs REQUIRED)

set(euclidean_cluster_executable euclidean_cluster)
set(euclidean_cluster_library ${euclidean_cluster_executable}_core)
set(dependencies
  rclcpp
  rclcpp_components
  pcl_conversions
  sensor_msgs
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(${euclidean_cluster_executable}
  src/euclidean_cluster_node.cpp
  src/euclidean_cluster_component.cpp
)
add_library(${euclidean_cluster_library} SHARED
  src/euclidean_cluster_component.cpp
)
ament_target_dependencies(${euclidean_cluster_executable}
  ${dependencies}
)
ament_target_dependencies(${euclidean_cluster_library}
  ${dependencies}
)
target_link_libraries(${euclidean_cluster_executable}
  ${PCL_LIBRARIES}
)
target_link_libraries(${euclidean_cluster_library}
  ${PCL_LIBRARIES}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
rclcpp_components_register_nodes(${euclidean_cluster_library}
  "navista_euclidean_cluster::EuclideanCluster"
)

install(TARGETS ${euclidean_cluster_executable}
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS ${euclidean_cluster_library}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_clang_format)
  ament_lint_auto_find_test_dependencies()
  ament_clang_format(
    CONFIG_FILE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../../.clang-format>
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(${euclidean_cluster_library})
ament_export_dependencies(${dependencies})
ament_package()
