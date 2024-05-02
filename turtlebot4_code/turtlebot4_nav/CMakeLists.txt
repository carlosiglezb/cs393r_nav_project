cmake_minimum_required(VERSION 3.8)
project(turtlebot4_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# create ROS2 nodes
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(amrl_msgs REQUIRED)
#find_package(irobot_create_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

#include_directories(include)

add_executable(battery_discharge_node src/simple_battery_discharge_node.cpp)
add_executable(cmd_vel_remap_node src/cmd_vel_remap_node.cpp)
add_executable(turtlebot4_recharge_monitor_node src/turtlebot4_recharge_monitor_node.cpp)
ament_target_dependencies(battery_discharge_node rclcpp sensor_msgs)
ament_target_dependencies(cmd_vel_remap_node rclcpp geometry_msgs)
ament_target_dependencies(turtlebot4_recharge_monitor_node
        rclcpp
        Eigen3
        sensor_msgs
        nav_msgs
        nav2_msgs
        std_msgs
        geometry_msgs
        amrl_msgs
#        irobot_create_msgs
        rclcpp_action)

target_include_directories(battery_discharge_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_include_directories(cmd_vel_remap_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_include_directories(turtlebot4_recharge_monitor_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(battery_discharge_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(cmd_vel_remap_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(turtlebot4_recharge_monitor_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS battery_discharge_node
        DESTINATION lib/${PROJECT_NAME})
install(TARGETS cmd_vel_remap_node
        DESTINATION lib/${PROJECT_NAME})
install(TARGETS turtlebot4_recharge_monitor_node
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
#install(DIRECTORY include/ DESTINATION include)

#ament_export_include_directories(include)
#ament_export_dependencies(sensor_msgs nav_msgs)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()