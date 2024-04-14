cmake_minimum_required(VERSION 3.10)

set(ROBOT_NAME "lite3")
project(${ROBOT_NAME}_motion_sdk)

message("CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
option(BUILD_PYTHON "Motion sdk python version" ON)

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
)

add_library(deeprobotics_${ROBOT_NAME}_motion_sdk_${CMAKE_SYSTEM_PROCESSOR} SHARED ${SRC_LIST})
target_link_libraries(deeprobotics_${ROBOT_NAME}_motion_sdk_${CMAKE_SYSTEM_PROCESSOR} -lpthread -lm -lrt -ldl -lstdc++)

add_custom_command(TARGET deeprobotics_${ROBOT_NAME}_motion_sdk_${CMAKE_SYSTEM_PROCESSOR} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:deeprobotics_${ROBOT_NAME}_motion_sdk_${CMAKE_SYSTEM_PROCESSOR}> ${CMAKE_BINARY_DIR}/../python/lib/
    COMMENT "Copying deeprobotics_${ROBOT_NAME}_motion_sdk_${CMAKE_SYSTEM_PROCESSOR} to python/lib"
)

add_executable(example_${ROBOT_NAME}_motion_sdk "main.cpp" "motionexample.h" "motionexample.cpp")
target_link_libraries(example_${ROBOT_NAME}_motion_sdk deeprobotics_${ROBOT_NAME}_motion_sdk_${CMAKE_SYSTEM_PROCESSOR})

if(BUILD_PYTHON)
  add_subdirectory(python)
endif()
