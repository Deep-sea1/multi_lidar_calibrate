cmake_minimum_required(VERSION 3.8)
project(multi_lidar_calibrator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(message_filters REQUIRED)
find_package(PCL REQUIRED)

set(CMAKE_CXX_STANDARD 17)

###########
## Build ##
###########

# MultiLidar Calibrator Library
add_library(multi_lidar_calibrator_lib SHARED
    src/multi_lidar_calibrator.cpp
)

target_include_directories(multi_lidar_calibrator_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(multi_lidar_calibrator_lib
    ${PCL_LIBRARIES}
)

ament_target_dependencies(multi_lidar_calibrator_lib
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    pcl_ros
    pcl_conversions
    tf2
    tf2_ros
    tf2_eigen
    message_filters
)

# MultiLidar Calibrator Node
add_executable(multi_lidar_calibrator
    src/multi_lidar_calibrator_node.cpp
)

target_include_directories(multi_lidar_calibrator PRIVATE
    include
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(multi_lidar_calibrator
    multi_lidar_calibrator_lib
    ${PCL_LIBRARIES}
)

ament_target_dependencies(multi_lidar_calibrator
    rclcpp
)

#############
## Install ##
#############

install(TARGETS
    multi_lidar_calibrator
    multi_lidar_calibrator_lib
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
    DESTINATION include
)

install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY cfg/
    DESTINATION share/${PROJECT_NAME}/cfg
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
