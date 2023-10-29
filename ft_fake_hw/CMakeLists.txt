cmake_minimum_required(VERSION 3.16)
project(ft_fake_hw)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(BUILD_SHARED_LIBS "Build shared libraries" ON)
find_package(ament_cmake REQUIRED)

set(DEPS
  rclcpp
  hardware_interface
  pluginlib
  realtime_tools
  geometry_msgs
)

foreach(dependency IN ITEMS ${DEPS})
  find_package(${dependency} REQUIRED)
endforeach()


add_library(
  ${PROJECT_NAME}
  src/ft_fake_hw.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${DEPS})
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


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

pluginlib_export_plugin_description_file(hardware_interface ft_fake_hw_plugin_description.xml)
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPS})
ament_package()
