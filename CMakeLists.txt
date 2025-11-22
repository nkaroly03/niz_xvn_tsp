cmake_minimum_required(VERSION 3.8)
project(niz_xvn_tsp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(ament_dependencies
  rclcpp
  geometry_msgs
  visualization_msgs
)

add_executable(tsp_node src/tsp.cpp)
ament_target_dependencies(tsp_node ${ament_dependencies} )

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
tsp_node
DESTINATION lib/${PROJECT_NAME}
)

ament_package()