cmake_minimum_required(VERSION 3.8)
project(ft_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)

# TODO(tpoignone): split in deps lib / deps nodes...
set(DEPENDENCIES
  rclcpp
  eigen3_cmake_module
  Eigen3
  tf2_eigen
  tf2_eigen_kdl
  kinematics_interface
  pluginlib
  realtime_tools
  yaml_cpp_vendor
  yaml-cpp
  ament_index_cpp
# msgs
  std_msgs
  geometry_msgs
  sensor_msgs
  std_srvs
  ft_msgs
)

foreach(Dependency IN ITEMS ${DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

# Wrench estimation/calibration library
add_library(${PROJECT_NAME} SHARED
  src/ft_calibration.cpp
  src/ft_estimation.cpp
  src/ft_parameters.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${YAML_CPP_INCLUDEDIR}
)
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})


# Calibration srv server (node)
add_executable(ft_calibration_srv_server
  src/ft_calibration_srv_server.cpp
)
ament_target_dependencies(ft_calibration_srv_server ${DEPENDENCIES})
target_include_directories(ft_calibration_srv_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(ft_calibration_srv_server PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(ft_calibration_srv_server ${PROJECT_NAME})

# Calibration node
add_executable(ft_calibration_node
  src/ft_calibration_node.cpp
  src/joint_state_monitor.cpp
)
ament_target_dependencies(ft_calibration_node PUBLIC ${DEPENDENCIES} generate_parameter_library)
target_include_directories(ft_calibration_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(ft_calibration_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
generate_parameter_library(ft_calibration_node_parameters
  src/ft_calibration_node_parameters.yaml
)
target_link_libraries(
  ft_calibration_node
  PUBLIC
    ${PROJECT_NAME}
    ft_calibration_node_parameters
)


# Estimation node
add_executable(ft_estimation_node
  src/ft_estimation_node.cpp
  src/joint_state_monitor.cpp
)
ament_target_dependencies(ft_estimation_node ${DEPENDENCIES})
target_include_directories(ft_estimation_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${YAML_CPP_INCLUDEDIR}
)
target_compile_features(ft_estimation_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(ft_estimation_node ${PROJECT_NAME})

# INSTALL
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(
    DIRECTORY include/
    DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS
    ft_calibration_srv_server
    ft_calibration_node
    ft_estimation_node
  DESTINATION
    lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
ament_export_dependencies(
  ${DEPENDENCIES}
)
ament_export_include_directories(
  include
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
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  #[===[
  # create custom test function to pass yaml file into test main
  function(add_gtest_with_parameters_gtest TARGET SOURCES YAML_FILE)
    add_executable(${TARGET} ${SOURCES})
    _ament_cmake_gtest_find_gtest()
    target_include_directories(${TARGET} PUBLIC "${GTEST_INCLUDE_DIRS}")
    target_link_libraries(${TARGET} ${GTEST_LIBRARIES})
    set(executable "$<TARGET_FILE:${TARGET}>")
    set(result_file "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${TARGET}.gtest.xml")
    ament_add_test(
      ${TARGET}
      COMMAND ${executable} --ros-args --params-file ${YAML_FILE} --
      --gtest_output=xml:${result_file}
      OUTPUT_FILE ${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${TARGET}.txt
      RESULT_FILE ${result_file}
    )
    endfunction()

  add_rostest_with_parameters_gtest(
    test_calibration
    "test/test_calibration.cpp;src/ft_calibration.cpp;src/ft_parameters.cpp"
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_asset_calibration_params.yaml
  )
  #]===]
  ament_add_gmock(
    test_calibration
    test/test_calibration.cpp
  )
  target_include_directories(
    test_calibration
    PUBLIC
    include test ${YAML_CPP_INCLUDEDIR}
  )
  ament_target_dependencies(test_calibration ${DEPENDENCIES})
  target_link_libraries(test_calibration ${PROJECT_NAME})
endif()

ament_package()
