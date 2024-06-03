cmake_minimum_required(VERSION 3.5)
project(radar_layer)

set(lib_name ${PROJECT_NAME}_core)

# === Environment ===

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

# === Dependencies ===

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(radar_msgs)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_dynamic_msgs REQUIRED)

set(dep_pkgs
    rclcpp
    nav2_costmap_2d
    pluginlib
    radar_msgs
    message_filters
    tf2_ros
    nav2_dynamic_msgs)

# === Build ===

include_directories(include)

add_library(${lib_name} SHARED
            src/radar_layer.cpp)

ament_target_dependencies(${lib_name} ${dep_pkgs})

# === Installation ===

install(TARGETS ${lib_name}
        DESTINATION lib)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

# === Ament work ===

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit Test
  find_package(Catch2 2 REQUIRED)
  set(test_dependencies
    Catch2
  )
  add_executable(catch_tests test/test_radar_layer.cpp)
  target_link_libraries(catch_tests Catch2::Catch2 ${lib_name})
  ament_target_dependencies(catch_tests
    ${test_dependencies}
    ${dep_pkgs}
  )
  add_test(NAME catch_tests COMMAND catch_tests)
endif()

# pluginlib_export_plugin_description_file() installs gradient_layer.xml
# file into "share" directory and sets ament indexes for it.
# This allows the plugin to be discovered as a plugin of required type.

pluginlib_export_plugin_description_file(nav2_costmap_2d radar_layer.xml)
ament_target_dependencies(${lib_name} ${dep_pkgs})
ament_package()
