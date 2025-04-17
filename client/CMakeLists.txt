# Minimum CMake required
cmake_minimum_required(VERSION 3.15)

# Project
project(realenv)

# vcpkg
set(VCPKG_HOME "/home/superfhwl/桌面/yaoxiang/dog_sim2real/0408_1/vcpkg/vcpkg")

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
set(hw_proto_srcs "${CMAKE_CURRENT_BINARY_DIR}/realenv.pb.cc")
set(hw_proto_hdrs "${CMAKE_CURRENT_BINARY_DIR}/realenv.pb.h")
set(hw_grpc_srcs "${CMAKE_CURRENT_BINARY_DIR}/realenv.grpc.pb.cc")
set(hw_grpc_hdrs "${CMAKE_CURRENT_BINARY_DIR}/realenv.grpc.pb.h")
# add_custom_command(
#       OUTPUT "${hw_proto_srcs}" "${hw_proto_hdrs}" "${hw_grpc_srcs}" "${hw_grpc_hdrs}"
#       COMMAND ${_PROTOBUF_PROTOC}
#       ARGS --grpc_out "${CMAKE_CURRENT_BINARY_DIR}"
#         --cpp_out "${CMAKE_CURRENT_BINARY_DIR}"
#         -I "${hw_proto_path}"
#         --plugin=protoc-gen-grpc="${_GRPC_CPP_PLUGIN_EXECUTABLE}"
#         "${hw_proto}"
#       DEPENDS "${hw_proto}")

# Include generated *.pb.h files
include_directories("${CMAKE_CURRENT_BINARY_DIR}")
include_directories("src")

# Add executable for the client
add_executable(client
    src/main.cpp
    src/client.cpp
    ${hw_proto_srcs}
    ${hw_grpc_srcs}
)

# Link libraries for the client
target_link_libraries(client
    ${_REFLECTION}
    ${_GRPC_GRPCPP}
    ${_PROTOBUF_LIBPROTOBUF}
)