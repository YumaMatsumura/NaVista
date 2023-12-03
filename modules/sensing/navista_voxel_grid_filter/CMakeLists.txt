cmake_minimum_required(VERSION 3.8)
project(navista_voxel_grid_filter)

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

set(voxel_grid_filter_executable voxel_grid_filter)
set(voxel_grid_filter_library ${voxel_grid_filter_executable}_core)
set(dependencies
  rclcpp
  rclcpp_components
  pcl_conversions
  sensor_msgs
)

include_directories(
  include
  ${PCL_INCLUD_DIRS}
)

add_executable(${voxel_grid_filter_executable}
  src/voxel_grid_filter_node.cpp
  src/voxel_grid_filter_component.cpp
)
add_library(${voxel_grid_filter_library} SHARED
  src/voxel_grid_filter_component.cpp
)
ament_target_dependencies(${voxel_grid_filter_executable}
  ${dependencies}
)
ament_target_dependencies(${voxel_grid_filter_library}
  ${dependencies}
)
target_link_libraries(${voxel_grid_filter_executable}
  ${PCL_LIBRARIES})
target_link_libraries(${voxel_grid_filter_library}
  ${PCL_LIBRARIES})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
rclcpp_components_register_nodes(${voxel_grid_filter_library}
  "navista_voxel_grid_filter::VoxelGridFilter"
)

install(TARGETS ${voxel_grid_filter_executable}
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS ${voxel_grid_filter_library}
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
ament_export_libraries(${voxel_grid_filter_library})
ament_export_dependencies(${dependencies})
ament_package()
