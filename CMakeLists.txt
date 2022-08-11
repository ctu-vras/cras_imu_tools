cmake_minimum_required(VERSION 3.0.2)
project(cras_imu_tools)

find_package(catkin REQUIRED COMPONENTS
  cras_cpp_common
  geometry_msgs
  nav_msgs
  nodelet
  sensor_msgs
  std_msgs
  topic_tools
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs imu_transformer sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(cras_gyro_bias_remover_nodelet src/gyro_bias_remover.cpp)
add_dependencies(cras_gyro_bias_remover_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cras_gyro_bias_remover_nodelet ${catkin_LIBRARIES})
cras_node_from_nodelet(cras_gyro_bias_remover_nodelet ${PROJECT_NAME}/gyro_bias_remover.h cras::GyroBiasRemoverNodelet OUTPUT_NAME gyro_bias_remover)

add_library(cras_transform_imu INTERFACE)
target_sources(cras_transform_imu INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/transform_imu.h>)
target_link_libraries(cras_transform_imu INTERFACE ${catkin_LIBRARIES})

install(TARGETS cras_gyro_bias_remover_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)