cmake_minimum_required(VERSION 3.8)
project(robotnik_charge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
add_compile_options(-std=c++17)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  std_srvs
  geometry_msgs
  robotnik_navigation_msgs
  robotnik_battery_msgs
  tf2_ros
  tf2_geometry_msgs
  generate_parameter_library
)
# find dependencies
find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(include)

generate_parameter_library(robotnik_charge_parameters
  src/robotnik_charge_params.yaml
  TARGET_NAMESPACE robotnik_charge
)

add_executable(robotnik_charge src/robotnik_charge.cpp src/robotnik_charge_node.cpp)

target_link_libraries(robotnik_charge robotnik_charge_parameters)

ament_target_dependencies(robotnik_charge
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

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

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  robotnik_charge
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
