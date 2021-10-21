cmake_minimum_required(VERSION 3.8)
project(gazebo_plugin_tutorials)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(gazebo REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

link_directories(${gazebo_dev_LIBRARY_DIRS})

# include_directories(${GAZEBO_INCLUDE_DIRS})
# link_directories(${GAZEBO_LIBRARY_DIRS})

# add_library(test_control_plugin SHARED src/test_control_plugin.cpp)
# target_link_libraries(test_control_plugin ${GAZEBO_LIBRARIES})

# add_executable(test_transport src/test_transport.cpp)
# target_link_libraries(test_transport ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_mechanum_control SHARED
  src/gazebo_ros_mechanum_control.cpp
)

target_include_directories(gazebo_ros_mechanum_control PUBLIC include)
ament_target_dependencies(gazebo_ros_mechanum_control
  "gazebo_dev"
  "gazebo_ros"
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
)
ament_export_libraries(gazebo_ros_mechanum_control)

ament_package()
