cmake_minimum_required(VERSION 3.9.5)
project(rplidar_ros2)

# Set the C++ standard to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Enable strict compile options for GCC/Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -pedantic)
endif()

# Set the build type to Release
set(CMAKE_BUILD_TYPE "Release")

# Define the path to the RPLIDAR SDK (located in the sdk/ folder)
set(RPLIDAR_SDK_PATH "${CMAKE_CURRENT_SOURCE_DIR}/sdk")

# Retrieve the SDK source files from various directories
file(GLOB RPLIDAR_SDK_SRC
  "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/*.cpp"
)

# List the required ROS2 dependencies
set(req_deps
  rclcpp
  sensor_msgs
  std_srvs
  rclcpp_components
)

# Find and configure ament packages
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_ros REQUIRED)
ament_auto_find_build_dependencies(REQUIRED ${req_deps})

# Include directories for the SDK and local headers
include_directories(
  ${RPLIDAR_SDK_PATH}/include
  ${RPLIDAR_SDK_PATH}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Build the composition node library from the node source and SDK sources
add_library(rplidar_composition_node SHARED
  src/rplidar_node.cpp
  ${RPLIDAR_SDK_SRC}
)
ament_target_dependencies(rplidar_composition_node ${req_deps})
rclcpp_components_register_nodes(rplidar_composition_node "rplidar_ros2::rplidar_node")

# Build the standalone executable using the composition node library
ament_auto_add_executable(rplidar_standalone src/rplidar_standalone.cpp)
target_link_libraries(rplidar_standalone rplidar_composition_node)
ament_target_dependencies(rplidar_standalone ${req_deps})

# Build the client executable (optional, as present in the project structure)
ament_auto_add_executable(rplidar_client src/rplidar_client.cpp)
target_link_libraries(rplidar_client rplidar_composition_node)
ament_target_dependencies(rplidar_client ${req_deps})

# Install the executables
install(
  TARGETS rplidar_standalone rplidar_client
  DESTINATION lib/${PROJECT_NAME}
)

# Install the composition node library
install(
  TARGETS rplidar_composition_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the launch, rviz, and scripts directories
install(
  DIRECTORY launch rviz scripts
  DESTINATION share/${PROJECT_NAME}
)

# Install public header files
install(
  DIRECTORY include/
  DESTINATION include
)

ament_auto_package()
