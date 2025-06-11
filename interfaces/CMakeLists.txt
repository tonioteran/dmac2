cmake_minimum_required(VERSION 3.8)
project(dmac2_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DMACAsync.msg"
  "msg/DMACClock.msg"
  "msg/DMACPayload.msg"
  "msg/DMACRaw.msg"
  "msg/DMACSync.msg"
  "msg/MUSBLFix.msg"
  DEPENDENCIES
   std_msgs
   sensor_msgs
   geometry_msgs
   builtin_interfaces
   diagnostic_msgs
 )
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
