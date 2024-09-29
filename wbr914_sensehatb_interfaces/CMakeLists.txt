cmake_minimum_required(VERSION 3.8)
project(wbr914_sensehatb_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED) ## For interfaces
# find_package(wbr914_sensehatb_interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Find the lgpio library
find_library(LGPIO_LIBRARY lgpio)

# Add custom interfaces - services(srv) and messages(msg) here
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Imu.msg"
  "msg/BaroPressure.msg"
  "msg/Temperature.msg"
  "msg/Ad.msg"
)

include_directories(
include/wbr914_sensehatb_interfaces
)

# Nodes executables
add_executable(wbr914_sensehatb_node
src/wbr914_sensehatb_node.cpp
src/LPS22HB.c 
src/SHTC3.c
src/ICM20948.c)

# target_link_libraries(wbr914_sensehatb_node)
# Link your executable to the lgpio library
if(LGPIO_LIBRARY)
    target_link_libraries(wbr914_sensehatb_node ${LGPIO_LIBRARY})
else()
    message(FATAL_ERROR "lgpio library not found")
endif()

ament_target_dependencies(wbr914_sensehatb_node rclcpp)
ament_export_dependencies(rosidl_default_runtime)
rosidl_target_interfaces(wbr914_sensehatb_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
wbr914_sensehatb_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
