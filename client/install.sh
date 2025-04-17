#!/bin/bash

# Define environment variables
PROTOC_PATH=/home/superfhwl/桌面/yaoxiang/dog_sim2real/0408_1/vcpkg/installed/x64-linux/tools/protobuf/protoc
GRPC_CPP_PLUGIN=/home/superfhwl/桌面/yaoxiang/dog_sim2real/0408_1/vcpkg/installed/x64-linux/tools/grpc/grpc_cpp_plugin

# Run the Protocol Buffers compiler
$PROTOC_PATH --cpp_out=build/ --grpc_out=build/ --plugin=protoc-gen-grpc=$GRPC_CPP_PLUGIN -I proto/ realenv.proto