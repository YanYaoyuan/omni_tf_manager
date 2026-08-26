cmake_minimum_required(VERSION 3.16)
project(omni_tf_manager)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
# Keep the interface surface explicit. The optional Humble Rust generator is
# deprecated and is not part of this C++/Python runtime contract.
find_package(rosidl_cmake REQUIRED)
find_package(rosidl_generator_c REQUIRED)
find_package(rosidl_generator_cpp REQUIRED)
find_package(rosidl_generator_py REQUIRED)
find_package(rosidl_typesupport_c REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_typesupport_fastrtps_c REQUIRED)
find_package(rosidl_typesupport_fastrtps_cpp REQUIRED)
find_package(rosidl_typesupport_introspection_c REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SlamStatus.msg"
  DEPENDENCIES builtin_interfaces)

add_library(${PROJECT_NAME}_core
  src/configuration.cpp
  src/transform_math.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_core Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME}_core
  geometry_msgs nav_msgs tf2 tf2_geometry_msgs)

add_executable(${PROJECT_NAME}_node src/omni_tf_manager_node.cpp)
target_include_directories(${PROJECT_NAME}_node PRIVATE include)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_core Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME}_node
  diagnostic_msgs geometry_msgs nav_msgs rclcpp sensor_msgs std_msgs
  tf2 tf2_geometry_msgs tf2_ros)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_node "${cpp_typesupport_target}")

install(TARGETS
  ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
# Executables are runtime artifacts, not linkable downstream libraries. Keep
# the node out of the exported target set so consumers of SlamStatus do not
# receive an invalid omni_tf_manager_node link dependency.
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(
  DIRECTORY config launch docs
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)
install(FILES README.md LICENSE DESTINATION share/${PROJECT_NAME})
install(PROGRAMS tools/validate_matrix_xgw_profile.py
  DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(geometry_msgs nav_msgs tf2 tf2_geometry_msgs)
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_transform_math test/test_transform_math.cpp)
  target_link_libraries(test_transform_math ${PROJECT_NAME}_core Eigen3::Eigen)
  ament_target_dependencies(test_transform_math geometry_msgs nav_msgs)

  ament_add_gtest(test_configuration test/test_configuration.cpp)
  target_link_libraries(test_configuration ${PROJECT_NAME}_core Eigen3::Eigen)
  ament_target_dependencies(test_configuration geometry_msgs)

  ament_add_gtest(test_sensor_frame_alias test/test_sensor_frame_alias.cpp)
  target_include_directories(test_sensor_frame_alias PRIVATE include)
  ament_target_dependencies(test_sensor_frame_alias sensor_msgs)

  ament_add_pytest_test(test_vbot_profile test/test_vbot_profile.py)

  # Use explicit source paths. ament_lint_auto recursively scans build/install
  # when this single-package repository is built from its own root directory.
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  ament_cppcheck(LANGUAGE c++ include src test)
  ament_cpplint(include src test)
  ament_uncrustify(include src test)
  ament_flake8(launch)
  ament_pep257(launch)
  ament_lint_cmake(CMakeLists.txt)
endif()

ament_package()
