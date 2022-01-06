cmake_minimum_required(VERSION 3.5)
project(roboticsgroup_upatras_gazebo_plugins)


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
find_package(rclcpp REQUIRED)
find_package(control_toolbox) # optionally take out
find_package(gazebo_ros)
find_package(gazebo REQUIRED)

include_directories(include)
link_directories(${gazebo_dev_LIBRARY_DIRS})


add_library(roboticsgroup_upatras_gazebo_mimic_joint_plugin SHARED
src/mimic_joint_plugin.cpp
)

ament_target_dependencies(roboticsgroup_upatras_gazebo_mimic_joint_plugin
  gazebo_dev
  gazebo_ros
  rclcpp
)


install(TARGETS
roboticsgroup_upatras_gazebo_mimic_joint_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(mimic_joint_plugin)

ament_export_dependencies(rclcpp)
ament_export_dependencies(gazebo_dev)
ament_export_dependencies(gazebo_ros)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


