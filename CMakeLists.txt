cmake_minimum_required(VERSION 3.8)
project(cr_pololu_maestro_servo_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


#
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cr_hand_interfaces REQUIRED)


#
# Collect SRC files
file(GLOB src_list ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)


#
# Servo Node
add_executable(cr_servo_node app/cr_servo_node.cpp ${src_list})
target_compile_features(cr_servo_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(cr_servo_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(cr_servo_node -lusb-1.0)
install(TARGETS cr_servo_node DESTINATION lib/${PROJECT_NAME})
ament_target_dependencies(cr_servo_node
  rclcpp
  sensor_msgs
  cr_hand_interfaces
)


#
# Test Node
add_library(test_actuate SHARED test/test_actuate.cpp)
target_compile_features(test_actuate PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(test_actuate
  rclcpp
  rclcpp_components
  sensor_msgs
  cr_hand_interfaces
)
rclcpp_components_register_node(
    test_actuate
    PLUGIN "TestMaestroNode"
    EXECUTABLE test_cr_servo_node
)
ament_export_targets(export_test_actuate)
install(TARGETS test_actuate
        EXPORT export_test_actuate
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)


#
# Package files
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()
