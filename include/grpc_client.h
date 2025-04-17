#ifndef CLIENT_H
#define CLIENT_H

#include <memory>
#include <grpcpp/grpcpp.h>
#include "realenv.grpc.pb.h"

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using realenv::PolicyService;
using realenv::Observation;
using realenv::Action;

class PolicyServiceClient {
public:
    explicit PolicyServiceClient(std::shared_ptr<grpc::Channel> channel);

    realenv::Action GetAction(const realenv::Observation& observation);

private:
    std::unique_ptr<realenv::PolicyService::Stub> stub_;
};

#endif // CLIENT_H