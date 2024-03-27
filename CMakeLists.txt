cmake_minimum_required(VERSION 3.8)
project(teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find processor Architecture
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
else()
  set(ARCH arm64)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ros2_unitree_legged_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


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

set(EXTRA_LIBS -pthread libunitree_legged_sdk.a)

# Add built Boost version 1.71
set(BOOST_ROOT "${CMAKE_SOURCE_DIR}/unitree_legged_sdk/boost_1_71_0")
set(Boost_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/unitree_legged_sdk/boost_1_71_0")
include_directories(${Boost_INCLUDE_DIRS}) 

include_directories(
    include
    include/${PROJECT_NAME}
    ${CMAKE_SOURCE_DIR}/unitree_legged_sdk/include
    ${sensor_msgs_INCLUDE_DIRS}
)

link_directories(${CMAKE_SOURCE_DIR}/unitree_legged_sdk/lib/cpp/${ARCH})

# Service and Message files
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PosCmd.srv"
 )

# Executables
add_executable(position_server src/position_server.cpp src/position_command.cpp)
target_link_libraries(position_server ${EXTRA_LIBS})
ament_target_dependencies(position_server rclcpp ros2_unitree_legged_msgs)

add_executable(joy_stick_teleop src/joy_controller.cpp src/vel_teleop.cpp)
target_link_libraries(joy_stick_teleop ${EXTRA_LIBS})
ament_target_dependencies(joy_stick_teleop rclcpp ros2_unitree_legged_msgs sensor_msgs)

install(TARGETS
    position_server
    joy_stick_teleop
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY ${CMAKE_SOURCE_DIR}/unitree_legged_sdk/lib/
    DESTINATION lib
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

  install(
    DIRECTORY ${CMAKE_SOURCE_DIR}/unitree_legged_sdk/lib/
    DESTINATION lib
)

ament_package()
