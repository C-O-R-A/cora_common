cmake_minimum_required(VERSION 3.8)
project(cora_moveit_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(ament_lint_auto REQUIRED)
find_package(Boost REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(cora_msgs REQUIRED)

if(BUILD_TESTING)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(cora_moveit_cpp
    src/move_group_test.cpp)
target_include_directories(cora_moveit_cpp
    PUBLIC include)
ament_target_dependencies(cora_moveit_cpp
    ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost
    rclcpp
    moveit_ros_planning_interface
    moveit_visual_tools
    geometry_msgs
    cora_msgs
    moveit_msgs)

install(TARGETS cora_moveit_cpp
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
target_link_libraries(cora_moveit_cpp
  Eigen3::Eigen
)

ament_package()

