cmake_minimum_required(VERSION 3.8)
project(navista_octomap_debug)

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
find_package(visualization_msgs REQUIRED)

set(octomap_debug_executable octomap_debug)
set(octomap_debug_library ${octomap_debug_executable}_core)
set(dependencies
  rclcpp
  rclcpp_components
  octomap
  octomap_msgs
  octomap_ros
  visualization_msgs
)

include_directories(include)

add_executable(${octomap_debug_executable}
  src/octomap_debug_node.cpp
  src/octomap_debug_component.cpp
)
add_library(${octomap_debug_library} SHARED
  src/octomap_debug_component.cpp
)
ament_target_dependencies(${octomap_debug_executable}
  ${dependencies}
)
ament_target_dependencies(${octomap_debug_library}
  ${dependencies}
)
rclcpp_components_register_nodes(${octomap_debug_library}
  "navista_octomap_debug::OctomapDebug"
)

install(TARGETS ${octomap_debug_executable}
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS ${octomap_debug_library}
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
ament_export_libraries(${octomap_debug_library})
ament_export_dependencies(${dependencies})
ament_package()
