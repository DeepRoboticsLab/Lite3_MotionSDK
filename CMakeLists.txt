cmake_minimum_required(VERSION 3.15)

project(Lite_motion)
SET(BUILD_PLATFORM "x86" CACHE STRING "select build cpu type")
if (BUILD_PLATFORM STREQUAL arm)
  message("this is arm platform")
  SET(CMAKE_C_COMPILER "aarch64-linux-gnu-gcc")
  SET(CMAKE_CXX_COMPILER "aarch64-linux-gnu-g++") 
else()
  message("this is x86 platform")
endif()

# vcpkg
set(VCPKG_HOME "../vcpkg/vcpkg")

# Protobuf
set(protobuf_MODULE_COMPATIBLE TRUE)
find_package(Protobuf CONFIG REQUIRED)
message(STATUS "Using protobuf ${protobuf_VERSION}")

# Protobuf-compiler
set(_PROTOBUF_PROTOC $<TARGET_FILE:protobuf::protoc>)

# gRPC
find_package(gRPC CONFIG REQUIRED)
message(STATUS "Using gRPC ${gRPC_VERSION}")
set(_GRPC_GRPCPP gRPC::grpc++)
set(_GRPC_CPP_PLUGIN_EXECUTABLE $<TARGET_FILE:gRPC::grpc_cpp_plugin>)

# Proto file
get_filename_component(hw_proto "realenv.proto" ABSOLUTE)
get_filename_component(hw_proto_path "${hw_proto}" PATH)

# Generated sources
set(hw_proto_srcs "build/realenv.pb.cc")
set(hw_proto_hdrs "build/realenv.pb.h")
set(hw_grpc_srcs "build/realenv.grpc.pb.cc")
set(hw_grpc_hdrs "build/realenv.grpc.pb.h")


set(CMAKE_BUILD_TYPE Release)
set(SRC_DIR_LIST "." src)
add_definitions(-w) # warning ignore
# set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

foreach(VAR ${SRC_DIR_LIST})
  set(TEMP)
  aux_source_directory(./src/ TEMP)
  set(SRC_LIST ${RCS_SRC_LIST} ${TEMP})
endforeach(VAR)

include_directories(
  ./include/
  ./include/common/
  ./lib/eigen3
  ./build
  # ./client/build
)

link_directories(
  ./lib/
)

add_executable(${PROJECT_NAME} 
"main.cpp" 
${SRC_LIST} 
${hw_proto_srcs}
 ${hw_grpc_srcs}
)
# 链接动态库target_link_libraries(myprogram /path/to/lib/libfoo.so)


 # 外部用cmake . -DBUILD_PLATFORM=arm进行值传入，便可以执行不同的逻辑
if (BUILD_PLATFORM STREQUAL arm)
  target_link_libraries(${PROJECT_NAME} libdeeprobotics_legged_sdk_aarch64.so)
else()
  target_link_libraries(${PROJECT_NAME} libdeeprobotics_legged_sdk_x86_64.so)
endif()

target_link_libraries(${PROJECT_NAME} -lpthread -lm -lrt -ldl -lstdc++)

target_link_libraries(${PROJECT_NAME}
    ${_REFLECTION}
    ${_GRPC_GRPCPP}
    ${_PROTOBUF_LIBPROTOBUF}
)


