cmake_minimum_required(VERSION 3.8)
project(navista_map_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(dependencies
  rosidl_default_generators
  rosidl_default_runtime
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/LoadMap.srv"
)

ament_export_dependencies(${dependencies})
ament_package()
