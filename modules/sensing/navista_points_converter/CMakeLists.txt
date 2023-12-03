cmake_minimum_required(VERSION 3.8)
project(navista_points_converter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(sensor_msgs REQUIRED)

set(points_converter_executable points_converter)
set(points_converter_library ${points_converter_executable}_core)
set(dependencies
  rclcpp
  rclcpp_components
  octomap
  octomap_msgs
  octomap_ros
  pcl_conversions
  sensor_msgs
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(${points_converter_executable}
  src/points_converter_node.cpp
  src/points_converter_component.cpp
)
add_library(${points_converter_library} SHARED
  src/points_converter_component.cpp
)
ament_target_dependencies(${points_converter_executable}
  ${dependencies}
)
ament_target_dependencies(${points_converter_library}
  ${dependencies}
)
target_link_libraries(${points_converter_executable}
  ${PCL_LIBRARIES}
)
target_link_libraries(${points_converter_library}
  ${PCL_LIBRARIES}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
rclcpp_components_register_nodes(${points_converter_library}
  "navista_points_converter::PointsConverter"
)

install(TARGETS ${points_converter_executable}
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS ${points_converter_library}
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
ament_export_libraries(${points_converter_library})
ament_export_dependencies(${dependencies})
ament_package()
