cmake_minimum_required(VERSION 3.8)
project(navista_map_loader)

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
find_package(navista_map_msgs REQUIRED)

set(map_loader_executable map_loader)
set(map_loader_library ${map_loader_executable}_core)
set(dependencies
  rclcpp
  rclcpp_components
  octomap
  octomap_msgs
  octomap_ros
  navista_map_msgs
)

include_directories(include)

add_executable(${map_loader_executable}
  src/map_loader_node.cpp
  src/map_loader_component.cpp
)
add_library(${map_loader_library} SHARED
  src/map_loader_component.cpp
)
ament_target_dependencies(${map_loader_executable}
  ${dependencies}
)
ament_target_dependencies(${map_loader_library}
  ${dependencies}
)
target_link_libraries(${map_loader_executable}
  yaml-cpp
)
target_link_libraries(${map_loader_library}
  yaml-cpp
)
rclcpp_components_register_nodes(${map_loader_library}
  "navista_map_loader::MapLoader"
)

install(TARGETS ${map_loader_executable}
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS ${map_loader_library}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION bin
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
ament_export_libraries(${map_loader_library})
ament_export_dependencies(${dependencies})
ament_package()
