cmake_minimum_required(VERSION 3.16)
project(robocell_follow)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_mppi_controller REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(robocell_follow SHARED target.cpp)
target_compile_features(robocell_follow PRIVATE cxx_std_17)
ament_target_dependencies(robocell_follow geometry_msgs nav2_mppi_controller pluginlib rclcpp)
pluginlib_export_plugin_description_file(nav2_mppi_controller critics.xml)
install(TARGETS robocell_follow DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_view test/view.cpp)
  if(TARGET test_view)
    target_include_directories(test_view PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  endif()
endif()

ament_package()
