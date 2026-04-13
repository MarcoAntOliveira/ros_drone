cmake_minimum_required(VERSION 3.8)
project(my_px4_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ##############################################################################
# Generate ROS messages, ROS2 interfaces and IDL files #
# ##############################################################################


# get all action files
set(ACTIONS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/action")
file(GLOB ACTIONS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${ACTIONS_DIR}/*.action")

# Generate introspection typesupport for C and C++ and IDL files
rosidl_generate_interfaces(${PROJECT_NAME}
	${ACTIONS}
	DEPENDENCIES builtin_interfaces
	ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
