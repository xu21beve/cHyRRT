cmake_minimum_required(VERSION 3.8)
project(visualize)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(Qt5 REQUIRED COMPONENTS Widgets)
# find_package(rclcpp REQUIRED)
# find_package(rclcpp_action REQUIRED)
# find_package(rosidl_default_generators REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(std_srvs REQUIRED)
# find_package(visualization_msgs REQUIRED)
find_package(ament_lint_auto REQUIRED)

include_directories(include ${Qt5Widgets_INCLUDE_DIRS})

# rosidl_generate_interfaces(${PROJECT_NAME}
#   "action/RotateAbsolute.action"
#   "msg/Color.msg"
#   "msg/Pose.msg"
#   "srv/Kill.srv"
#   "srv/SetPen.srv"
#   "srv/Spawn.srv"
#   "srv/TeleportAbsolute.srv"
#   "srv/TeleportRelative.srv")

set(dependencies "ament_index_cpp" "geometry_msgs" "rclcpp" "rclcpp_action" "std_msgs" "std_srvs" "visualization_msgs")

# set(turtlesim_node_SRCS
#   src/turtlesim.cpp
#   src/turtle.cpp
#   src/turtle_frame.cpp
# )
# 
# set(turtlesim_node_HDRS
#   include/turtlesim/turtle_frame.h
# )

# qt5_wrap_cpp(turtlesim_node_MOCS ${turtlesim_node_HDRS})

# rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME})

add_executable(ros_visualize src/ros_visualize.cpp)
target_link_libraries(ros_visualize Qt5::Widgets)
ament_target_dependencies(ros_visualize ${dependencies})
# target_link_libraries(ros_visualize "${cpp_typesupport_target}")

# add_executable(turtlesim_node ${turtlesim_node_SRCS} ${turtlesim_node_MOCS})
# target_link_libraries(turtlesim_node Qt5::Widgets)
# ament_target_dependencies(turtlesim_node ${dependencies})
# target_link_libraries(turtlesim_node "${cpp_typesupport_target}")

# add_executable(turtle_teleop_key tutorials/teleop_turtle_key.cpp)
# ament_target_dependencies(turtle_teleop_key ${dependencies})
# target_link_libraries(turtle_teleop_key "${cpp_typesupport_target}")
# add_executable(draw_square tutorials/draw_square.cpp)
# ament_target_dependencies(draw_square ${dependencies})
# target_link_libraries(draw_square "${cpp_typesupport_target}")
# add_executable(mimic tutorials/mimic.cpp)
# ament_target_dependencies(mimic ${dependencies})
# target_link_libraries(mimic "${cpp_typesupport_target}")

install(TARGETS
  ros_visualize
  DESTINATION lib/${PROJECT_NAME})

# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
