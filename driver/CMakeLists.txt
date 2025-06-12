cmake_minimum_required(VERSION 3.8)
project(dmac2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(dmac2_interfaces REQUIRED)
find_package(Boost COMPONENTS thread system regex REQUIRED)

# TODO: this seems to be a ROS1 thing...
#find_package(message_generation REQUIRED)

add_executable(dmac src/dmac_driver.cpp)
target_link_libraries(dmac Boost::thread Boost::regex)
ament_target_dependencies(dmac rclcpp std_msgs dmac2_interfaces)

install(TARGETS
    dmac
    DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

