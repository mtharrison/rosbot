cmake_minimum_required(VERSION 3.8)
project(motor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_library(wiringPiLib wiringPi) 

add_executable(motor 
  src/main.cpp 
  src/pid.cpp 
  src/motor.cpp 
  src/encoder.cpp 
  src/kinematics.cpp 
  src/odometry.cpp)

ament_target_dependencies(
  motor 
  rclcpp 
  std_msgs 
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros)
target_link_libraries(motor ${wiringPiLib})
target_include_directories(motor PUBLIC include)

install(TARGETS motor DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
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

ament_package()
