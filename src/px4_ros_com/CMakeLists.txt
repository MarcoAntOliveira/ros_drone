cmake_minimum_required(VERSION 3.5)
project(px4_ros_com)

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
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(my_px4_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)

#################
# Setup targets #
#################

include_directories(include)

# Add frame_transforms lib
add_library(frame_transforms SHARED src/lib/frame_transforms.cpp)
ament_target_dependencies(frame_transforms Eigen3 geometry_msgs sensor_msgs)
target_include_directories(frame_transforms PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
	$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)


# examples/listeners/sensor_combined_listener
add_executable(sensor_combined_listener src/examples/listeners/sensor_combined_listener.cpp)
ament_target_dependencies(sensor_combined_listener rclcpp px4_msgs)
install(TARGETS sensor_combined_listener DESTINATION lib/${PROJECT_NAME})

# examples/listeners/vehicle_gps_position_listener
add_executable(vehicle_gps_position_listener src/examples/listeners/vehicle_gps_position_listener.cpp)
ament_target_dependencies(vehicle_gps_position_listener rclcpp px4_msgs)
install(TARGETS vehicle_gps_position_listener DESTINATION lib/${PROJECT_NAME})

# examples/advertisers/debug_vect_advertiser
add_executable(debug_vect_advertiser src/examples/advertisers/debug_vect_advertiser.cpp)
ament_target_dependencies(debug_vect_advertiser rclcpp px4_msgs)
install(TARGETS debug_vect_advertiser DESTINATION lib/${PROJECT_NAME})

# examples/offboard/offboard_control
add_executable(offboard_control src/examples/offboard/offboard_control.cpp)
ament_target_dependencies(offboard_control rclcpp px4_msgs my_px4_interfaces rclcpp_action builtin_interfaces)
install(TARGETS offboard_control DESTINATION lib/${PROJECT_NAME})


#swarm drones
add_executable(offboard1 src/examples/offboard/offboard1.cpp)
ament_target_dependencies(offboard1 rclcpp px4_msgs my_px4_interfaces rclcpp_action builtin_interfaces)
install(TARGETS offboard1 DESTINATION lib/${PROJECT_NAME})

add_executable(offboard2 src/examples/offboard/offboard2.cpp)
ament_target_dependencies(offboard2 rclcpp px4_msgs my_px4_interfaces rclcpp_action builtin_interfaces)
install(TARGETS offboard2 DESTINATION lib/${PROJECT_NAME})

add_executable(offboard3 src/examples/offboard/offboard3.cpp)
ament_target_dependencies(offboard3 rclcpp px4_msgs my_px4_interfaces rclcpp_action builtin_interfaces)
install(TARGETS offboard3 DESTINATION lib/${PROJECT_NAME})

add_executable(offboard4 src/examples/offboard/offboard4.cpp)
ament_target_dependencies(offboard4 rclcpp px4_msgs my_px4_interfaces rclcpp_action builtin_interfaces)
install(TARGETS offboard4 DESTINATION lib/${PROJECT_NAME})

add_executable(offboard5 src/examples/offboard/offboard5.cpp)
ament_target_dependencies(offboard5 rclcpp px4_msgs my_px4_interfaces rclcpp_action builtin_interfaces)
install(TARGETS offboard5 DESTINATION lib/${PROJECT_NAME})

add_executable(offboard_vel src/missions/offboard_vel.cpp)
ament_target_dependencies(offboard_vel rclcpp px4_msgs my_px4_interfaces rclcpp_action builtin_interfaces geometry_msgs)
install(TARGETS offboard_vel DESTINATION lib/${PROJECT_NAME})


add_executable(offboard_control2 src/examples/offboard/offboard_control.cpp)
ament_target_dependencies(offboard_control2 rclcpp px4_msgs rclcpp_action builtin_interfaces)
install(TARGETS offboard_control2 DESTINATION lib/${PROJECT_NAME})

add_executable(mission3 src/missions/mission3.cpp)
ament_target_dependencies(mission3 rclcpp px4_msgs)
install(TARGETS mission3 DESTINATION lib/${PROJECT_NAME})

add_executable(main src/missions/main.cpp  src/missions/offboard.cpp)
ament_target_dependencies(main rclcpp px4_msgs rclcpp_action  my_px4_interfaces  builtin_interfaces)
install(TARGETS main DESTINATION lib/${PROJECT_NAME})

# examples/offboard/offboard_control_srv
add_executable(offboard_control_srv src/examples/offboard/offboard_control_srv.cpp)
ament_target_dependencies(offboard_control_srv rclcpp px4_msgs)
install(TARGETS offboard_control_srv DESTINATION lib/${PROJECT_NAME})


############
# Install ##
############

# Export information to downstream packages
ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime eigen3_cmake_module Eigen3 px4_msgs my_px4_interfaces geometry_msgs sensor_msgsj)

ament_export_targets(export_frame_transforms HAS_LIBRARY_TARGET)

ament_export_include_directories(include)
ament_export_libraries(frame_transforms)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

install(TARGETS frame_transforms
	EXPORT export_frame_transforms
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
	INCLUDES DESTINATION include
)

# Install launch files.
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# Install tests
install(DIRECTORY test DESTINATION share/${PROJECT_NAME}/)


############
# Testing ##
############

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

###########
# Python ##
###########

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  src/examples/offboard_py/offboard_control.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  src/missions/detect_aruco.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
