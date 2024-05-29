cmake_minimum_required(VERSION 3.8)
project(cpp_pubsub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED) 
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)


add_executable(subNode src/sub.cpp)
ament_target_dependencies(subNode rclcpp std_msgs)

add_executable(turNode src/time_pass.cpp)
ament_target_dependencies(turNode rclcpp turtlesim tf2_geometry_msgs tf2 sensor_msgs nav_msgs geometry_msgs std_msgs)

add_executable(pubsubNode src/pubsub.cpp)
ament_target_dependencies(pubsubNode rclcpp turtlesim geometry_msgs std_msgs)

add_executable(tb3Node src/first_node.cpp)
ament_target_dependencies(tb3Node rclcpp turtlesim tf2_geometry_msgs tf2 sensor_msgs nav_msgs geometry_msgs std_msgs)

add_executable(ttlBotNode src/tb_vyom.cpp)
ament_target_dependencies(ttlBotNode rclcpp turtlesim tf2_geometry_msgs tf2 sensor_msgs nav_msgs geometry_msgs std_msgs)

add_executable(ps4Node src/ps4.cpp)
ament_target_dependencies(ps4Node rclcpp sensor_msgs std_msgs geometry_msgs)

add_executable(pstryNode src/ps4_try.cpp)
ament_target_dependencies(pstryNode rclcpp sensor_msgs std_msgs geometry_msgs)

add_executable(tryNode src/tb_final.cpp)
ament_target_dependencies(tryNode rclcpp turtlesim tf2_geometry_msgs tf2 sensor_msgs nav_msgs geometry_msgs std_msgs)

add_executable(t2Node src/try2.cpp)
ament_target_dependencies(t2Node rclcpp turtlesim tf2_geometry_msgs tf2 sensor_msgs nav_msgs geometry_msgs std_msgs)

add_executable(finalNode src/g2g_obs_avoidance_software.cpp)
ament_target_dependencies(finalNode rclcpp turtlesim tf2_geometry_msgs tf2 sensor_msgs nav_msgs geometry_msgs std_msgs)

install(TARGETS
  subNode
  finalNode
  turNode
  pubsubNode
  tb3Node
  ttlBotNode
  ps4Node
  pstryNode
  tryNode
  t2Node
  DESTINATION lib/${PROJECT_NAME})


# uncomment the followinps4g section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

ament_package()
