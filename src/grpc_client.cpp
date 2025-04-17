#include "grpc_client.h"
#include <iostream>

PolicyServiceClient::PolicyServiceClient(std::shared_ptr<grpc::Channel> channel)
    : stub_(PolicyService::NewStub(channel)) {}

realenv::Action PolicyServiceClient::GetAction(const realenv::Observation& observation) {
    realenv::Action action;
    grpc::ClientContext context;

    grpc::Status status = stub_->GetAction(&context, observation, &action);

    if (status.ok()) {
        return action;
    } else {
        std::cerr << "gRPC call failed: " << status.error_message() << std::endl;
        return realenv::Action();
    }
}