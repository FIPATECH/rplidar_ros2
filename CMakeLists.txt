cmake_minimum_required(VERSION 3.9.5)
project(rplidar_ros2)

# Set the C++ standard to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Enable strict compile options for GCC/Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -pedantic -Wno-pedantic)
endif()

# Define the path to the RPLIDAR SDK
set(RPLIDAR_SDK_PATH "./sdk/")

# Collect all source files from the RPLIDAR SDK
file(GLOB RPLIDAR_SDK_SRC
  "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/*.cpp"
)

# List the required ROS2 dependencies
set(req_deps
  "rclcpp"
  "sensor_msgs"
  "std_srvs"
  "rclcpp_components"
)

# Find and configure ament packages
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_ros REQUIRED)

# Automatically find and include required ROS2 dependencies
ament_auto_find_build_dependencies(REQUIRED ${req_deps})

# Include directories for the SDK and local headers
include_directories(
  ${RPLIDAR_SDK_PATH}/include
  ${RPLIDAR_SDK_PATH}/src
  include
)

# Build the shared library for the rplidar_node component (ROS2 node)
add_library(rplidar_node_node SHARED
  src/rplidar_node.cpp
  ${RPLIDAR_SDK_SRC}
)

# Link the rplidar_node library with the required dependencies
ament_target_dependencies(rplidar_node_node ${req_deps})

# Register the rplidar_node component so it can be used in ROS2 composition
rclcpp_components_register_nodes(rplidar_node_node "rplidar_ros2::rplidar_node")

# Build the standalone executable for the rplidar node
ament_auto_add_executable(rplidar_node src/rplidar_standalone.cpp)

# Link the standalone executable to the rplidar_node component library
target_link_libraries(rplidar_node rplidar_node_node)

# Link the required ROS2 dependencies
ament_target_dependencies(rplidar_node ${req_deps})

# Install the standalone executable
install(
  TARGETS rplidar_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch and RViz configuration files
install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install the rplidar_node shared library (component)
install(
  TARGETS rplidar_node_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Finalize the ament package configuration
ament_auto_package()
