cmake_minimum_required(VERSION 3.8.0)
project(mechai_sims)

# Default to C11
# if(NOT CMAKE_C_STANDARD)
#   set(CMAKE_C_STANDARD 11)
# endif()
# # Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   # we dont use add_compile_options with pedantic in message packages
#   # because the Python C extensions dont comply with it
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
# endif()

######################Dependencies#############################
# find dependencies
# find_package(ament_cmake REQUIRED)

# find_package(rclcpp REQUIRED)
# find_package(rclpy REQUIRED)

# find_package(control_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(std_msgs REQUIRED)

# find_package(xacro)

# find_package(gazebo_dev REQUIRED)
# find_package(gazebo_msgs REQUIRED)
# find_package(gazebo_ros REQUIRED)

# find_package(geometry_msgs REQUIRED)
# find_package(nav_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(std_srvs REQUIRED)

# find_package(tf2 REQUIRED)
# find_package(tf2_geometry_msgs REQUIRED)
# find_package(tf2_ros REQUIRED)

######################Install util directories################
# install(DIRECTORY
#   launch
#   urdf
#   config
#   meshes
#   world
#   DESTINATION share/${PROJECT_NAME}/
# )

######################CPP Part#############################
# include_directories(include)

# link_directories(${gazebo_dev_LIBRARY_DIRS})

# add_library(gazebo_ros_mecanum_control SHARED
#   src/gazebo_ros_mecanum_control.cpp
# )

# target_include_directories(gazebo_ros_mecanum_control PUBLIC include)
# ament_target_dependencies(gazebo_ros_mecanum_control
#   "gazebo_dev"
#   "gazebo_ros"
#   "geometry_msgs"
#   "nav_msgs"
#   "rclcpp"
#   "tf2"
#   "tf2_geometry_msgs"
#   "tf2_ros"
# )



#export libraries
# ament_export_libraries(gazebo_ros_mecanum_control)

# Install Cpp executables
# install(TARGETS
#   cpp_executable
#   DESTINATION lib/${PROJECT_NAME}
# )

# nav2 tutorial
# install(
#   DIRECTORY src launch rviz config world
#   DESTINATION share/${PROJECT_NAME}
# )

# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)
#   find_package(ament_lint_auto REQUIRED)

#   ament_lint_auto_find_test_dependencies()
# endif()

######################Python Part#############################
# Install Python modules
# ament_python_install_package(${PROJECT_NAME})

# # Install Python executables
# install(PROGRAMS
#   scripts/state_publisher.py
#   DESTINATION lib/${PROJECT_NAME}
# )



########################gazebo ros2 control######################
# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(NOT WIN32)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)

install(DIRECTORY
  launch
  config
  urdf
  world
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

