cmake_minimum_required(VERSION 3.8)
project(my_nav2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(nav_msgs REQUIRED) 
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(odometry_pub src/odometry_pub.cpp)

ament_target_dependencies(odometry_pub rclcpp nav_msgs std_msgs geometry_msgs tf2_geometry_msgs tf2)
target_link_libraries(odometry_pub
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
)

install(TARGETS
  odometry_pub
  DESTINATION lib/${PROJECT_NAME}
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

install(
  DIRECTORY
    launch
    maps
    params
    config
  DESTINATION
    share/${PROJECT_NAME}/
)

install(
  PROGRAMS
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/my_nav_cmd.py
  scripts/my_odom.py
  scripts/marker.py
  scripts/map.py


  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
