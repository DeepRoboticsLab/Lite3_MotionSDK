cmake_minimum_required(VERSION 3.10)

set(ROBOT_NAME "lite3")
project(${ROBOT_NAME}_motion_sdk)

message("CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")

set(CMAKE_BUILD_TYPE Release)
set(SRC_DIR_LIST "." src)
add_definitions(-w) # warning ignore
set(CMAKE_CXX_FLAGS "-std=c++11 ${CAMKE_CXX_FLAGS}")

foreach(VAR ${SRC_DIR_LIST})
  set(TEMP)
  aux_source_directory(./src/ TEMP)
  set(SRC_LIST ${RCS_SRC_LIST} ${TEMP})
endforeach(VAR)

include_directories(
  ./include/
  ./include/common/
  # ./lib/eigen3
)

link_directories(
  ./lib/
)

add_executable(deeprobotics_${ROBOT_NAME}_motion_sdk_${CMAKE_SYSTEM_PROCESSOR} "main.cpp" ${SRC_LIST})
target_link_libraries(deeprobotics_${ROBOT_NAME}_motion_sdk_${CMAKE_SYSTEM_PROCESSOR} -lpthread -lm -lrt -ldl -lstdc++)


