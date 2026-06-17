cmake_minimum_required(VERSION 3.8)
project(hello_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(hello_moveit 
src/hello_moveit.cpp
src/collision_objects.cpp)
add_executable(hello_moveit_cartesian
src/hello_moveit_cartesian.cpp
src/collision_objects.cpp
)
add_executable(print_current_pose src/print_current_pose.cpp)
add_executable(tool_jog src/tool_jog.cpp)
target_include_directories(hello_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_include_directories(hello_moveit_cartesian PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(hello_moveit PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(hello_moveit_cartesian PUBLIC c_std_99 cxx_std_17)
target_compile_features(print_current_pose PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(tool_jog PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  hello_moveit
  "moveit_ros_planning_interface"
  "rclcpp"
  "moveit_visual_tools"
)

ament_target_dependencies(
  hello_moveit_cartesian
  "moveit_ros_planning_interface"
  "rclcpp"
  "moveit_visual_tools"
)

ament_target_dependencies(
  print_current_pose
  "moveit_ros_planning_interface"
  "rclcpp"
)

ament_target_dependencies(
  tool_jog
  "moveit_ros_planning_interface"
  "rclcpp"
  "tf2"
  "tf2_geometry_msgs"
)

install(TARGETS 
  hello_moveit 
  hello_moveit_cartesian
  print_current_pose
  tool_jog
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

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
