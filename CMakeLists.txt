cmake_minimum_required(VERSION 3.8)
project(g1_nav)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

add_executable(
  frontier_explorer_cpp
  src/frontier_explorer.cpp
  src/frontier_search.cpp
  src/gvd_map.cpp
)
target_compile_features(frontier_explorer_cpp PUBLIC cxx_std_17)
target_include_directories(frontier_explorer_cpp PUBLIC include)
ament_target_dependencies(
  frontier_explorer_cpp
  geometry_msgs
  nav2_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  std_msgs
  tf2_msgs
  visualization_msgs
)

install(DIRECTORY
  config
  include
  launch
  rviz
  semantic
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  g1_nav/frontier_explorer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME frontier_explorer.py
)

install(PROGRAMS
  g1_nav/nav2_precheck.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME nav2_precheck
)

install(PROGRAMS
  g1_nav/tf_to_odom.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME tf_to_odom
)

install(PROGRAMS
  g1_nav/map_cache_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME map_cache_bridge
)

install(PROGRAMS
  g1_nav/map_warmup_spin.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME map_warmup_spin
)

install(TARGETS
  frontier_explorer_cpp
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
