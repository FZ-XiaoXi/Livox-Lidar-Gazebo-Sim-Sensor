cmake_minimum_required(VERSION 3.22.1)
project(livox_lidar_gz_sensor)

find_package(ament_cmake REQUIRED)

# 1. 必须先找到 gz-cmake3，才能使用 gz_find_package
find_package(gz-cmake3 REQUIRED)

# 2. 然后才能查找具体的 Gazebo 依赖
gz_find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

gz_find_package(gz-common5 REQUIRED)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

gz_find_package(gz-sensors8 REQUIRED)
set(GZ_SENSORS_VER ${gz-sensors8_VERSION_MAJOR})

gz_find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

gz_find_package(gz-msgs10 REQUIRED)
set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})

gz_find_package(gz-transport13 REQUIRED)
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})

# 3. 创建插件库并链接
add_library(${PROJECT_NAME} SHARED
  src/livox_lidar_system.cpp
  src/livox_lidar_sensor.cpp
)
target_link_libraries(${PROJECT_NAME} PRIVATE
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  gz-sensors${GZ_SENSORS_VER}::gz-sensors${GZ_SENSORS_VER}
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
  gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

install(
  DIRECTORY urdf scan_mode meshes
  DESTINATION share/${PROJECT_NAME}
)

# Add this package's shared-library directory to Gazebo's system-plugin search
# path whenever the workspace environment is sourced.
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in"
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in"
)

ament_package()
