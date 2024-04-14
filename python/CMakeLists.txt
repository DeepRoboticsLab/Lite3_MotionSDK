cmake_minimum_required(VERSION 3.10)
project(deeprobotics_${ROBOT_NAME}_motion_sdk_py)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -O3 -fPIC")
set(CMAKE_CXX_STANDARD 11)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

add_subdirectory(pybind11)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)

pybind11_add_module(${PROJECT_NAME} motion_sdk_pybind.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE -lpthread -lm -lrt -ldl -lstdc++ deeprobotics_${ROBOT_NAME}_motion_sdk_${CMAKE_SYSTEM_PROCESSOR})
