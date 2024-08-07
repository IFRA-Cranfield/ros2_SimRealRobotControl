cmake_minimum_required(VERSION 3.8)
project(ros2srrc_execution)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
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

# =========================================================== #
# REQUIRED TO EXECUTE .cpp scripts -> move.cpp:

# REQUIRED to -> Launch move_group interface:
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

# REQUIRED to -> Include ros2srrc_data package:
find_package(ros2srrc_data REQUIRED)

# REQUIRED to -> Include linkattacher_msgs package:
find_package(linkattacher_msgs REQUIRED)

# REQUIRED to -> Execute ACTIONS:
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)

# REQUIRED to -> Read YAML files:
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

# REQUIRED to -> ABB:
find_package(abb_robot_msgs REQUIRED)

# REQUIRED to -> Robotiq:
find_package(ros2_robotiqgripper REQUIRED)

# Add include directories:
include_directories(include)

# Add executable:
add_executable(
  move
  src/moveg.cpp
  src/moverp.cpp
  src/moverot.cpp
  src/mover.cpp
  src/movel.cpp
  src/movej.cpp
  src/move.cpp
)
add_executable(
  robmove
  src/robmove.cpp
)
add_executable(
  robpose
  src/robpose.cpp
)

# Link directories:
include_directories(${YAML_CPP_INCLUDE_DIR})
target_link_libraries(move ${YAML_CPP_LIBRARIES})

# Install executable:
install(TARGETS
  move
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  robmove
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  robpose
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files:
install(
  DIRECTORY include/
  DESTINATION include/
)

# Add ament dependencies:
ament_target_dependencies(
  move
  rclcpp
  moveit_ros_planning_interface
  std_msgs
  rclcpp_action
  ros2srrc_data
  abb_robot_msgs
  yaml-cpp
  ament_index_cpp
)
ament_target_dependencies(
  robmove
  rclcpp
  moveit_ros_planning_interface
  std_msgs
  rclcpp_action
  ros2srrc_data
)
ament_target_dependencies(
  robpose
  rclcpp
  moveit_ros_planning_interface
  std_msgs
  ros2srrc_data
)

# =========================================================== #
# REQUIRED TO EXECUTE .py scripts -> sequence.py:
install(
  DIRECTORY 
    python
    programs
  DESTINATION 
    share/${PROJECT_NAME}
)
install(PROGRAMS
  
  python/sequence.py
  python/RobotState.py
  python/SpawnObject.py
  python/ExecuteProgram.py
  
  python/testClients/test_Move.py
  python/testClients/test_RobMove.py
  python/testClients/test_PG.py
  python/testClients/test_VG.py

  DESTINATION lib/${PROJECT_NAME} 
)

ament_package()