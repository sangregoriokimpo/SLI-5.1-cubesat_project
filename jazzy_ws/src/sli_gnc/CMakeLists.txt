cmake_minimum_required(VERSION 3.8)
project(sli_gnc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(orbit_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(simulation_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

set(DEPS
  rclcpp
  orbit_interfaces
  std_msgs
  simulation_interfaces
)


add_executable(teleop_node src/teleop_node.cpp)
ament_target_dependencies(teleop_node ${DEPS})

add_executable(orbit_vel_node src/orbit_vel_node.cpp)
ament_target_dependencies(orbit_vel_node ${DEPS})

add_executable(cw_node src/cw_node.cpp)
ament_target_dependencies(cw_node ${DEPS})

add_executable(isaac_bridge_node src/isaac_bridge_node.cpp)
ament_target_dependencies(isaac_bridge_node ${DEPS})

install(TARGETS teleop_node
  teleop_node
  orbit_vel_node
  cw_node
  isaac_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
