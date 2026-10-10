# SPDX-FileCopyrightText: 2026 FZ_XiaoXi
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.22.1)
project(livox_lidar_example)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(livox_lidar_gz_sensor REQUIRED)

file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/worlds")

xacro_add_xacro_file(
  worlds/livox_example.world.xacro
  "${CMAKE_CURRENT_BINARY_DIR}/worlds/livox_example.sdf"
)

add_custom_target(generate_livox_example_world ALL
  DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/worlds/livox_example.sdf"
)

install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/worlds/livox_example.sdf"
  DESTINATION share/${PROJECT_NAME}/worlds
)

install(
  FILES
    launch/livox_example.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY
    config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    run_livox_example.sh
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
