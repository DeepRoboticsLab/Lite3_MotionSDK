#!/bin/bash

# Define environment variables
PROTOC_PATH=../vcpkg/installed/x64-linux/tools/protobuf/protoc
GRPC_CPP_PLUGIN=../vcpkg/installed/x64-linux/tools/grpc/grpc_cpp_plugin

# Run the Protocol Buffers compiler
$PROTOC_PATH --cpp_out=build/ --grpc_out=build/ --plugin=protoc-gen-grpc=$GRPC_CPP_PLUGIN -I include/proto/ realenv.proto