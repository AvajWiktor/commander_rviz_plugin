cmake_minimum_required(VERSION 3.5)
project(commander_rviz_plugins)

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
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Qt5 REQUIRED COMPONENTS Widgets)
set(CMAKE_AUTOMOC ON)

set(commander_rviz_plugins_headers_to_moc
  include/commander_rviz_plugins/commander_display.hpp
)

foreach(header "${commander_rviz_plugins_headers_to_moc}")
  qt5_wrap_cpp(commander_rviz_plugins_moc_files "${header}")
endforeach()

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${commander_rviz_plugins_moc_files}
  src/commander_display.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
  "${PROJECT_SOURCE_DIR}/include"
)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

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

ament_auto_package()