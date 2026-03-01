cmake_minimum_required(VERSION 3.16)
project(zenorak_controller_motor LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
  zenorak_controller_motor
  SHARED
  hardware/diffbot_system.cpp
)
target_compile_features(zenorak_controller_motor PUBLIC cxx_std_17)
target_include_directories(zenorak_controller_motor PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/zenorak_controller_motor>
)
ament_target_dependencies(
  zenorak_controller_motor PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(zenorak_controller_motor PUBLIC serial)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "ZENORAK_CONTROLLER_MOTOR_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface zenorak_controller_motor.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/zenorak_controller_motor
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf description/rviz
  DESTINATION share/zenorak_controller_motor
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/zenorak_controller_motor
)
install(TARGETS zenorak_controller_motor
  EXPORT export_zenorak_controller_motor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_zenorak_controller_motor HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
