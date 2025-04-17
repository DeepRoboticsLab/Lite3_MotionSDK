#include <iostream>
#include "header/client.h"

int main(int argc, char** argv) {
    // Create a client connected to the server
    PolicyServiceClient client(grpc::CreateChannel("localhost:50051", grpc::InsecureChannelCredentials()));

    // Create a fake observation
    Observation observation;
    observation.set_tick(12345);

    // Set IMU data
    auto* imu = observation.mutable_imu();
    imu->set_angle_roll(1.0);
    imu->set_angle_pitch(2.0);
    imu->set_angle_yaw(3.0);
    imu->set_angular_velocity_roll(0.1);
    imu->set_angular_velocity_pitch(0.2);
    imu->set_angular_velocity_yaw(0.3);
    imu->set_acc_x(9.8);
    imu->set_acc_y(0.0);
    imu->set_acc_z(-9.8);

    // Set joint data for each leg
    auto* fl_leg = observation.mutable_joint_data()->mutable_fl_leg();
    for (int i = 0; i < 3; ++i) {
        auto* joint = fl_leg->Add();
        joint->set_position(0.1 + i * 0.3);
        joint->set_velocity(0.2 + i * 0.3);
        joint->set_torque(0.3 + i * 0.3);
        joint->set_temperature(40.0 + i);
    }

    auto* fr_leg = observation.mutable_joint_data()->mutable_fr_leg();
    for (int i = 0; i < 3; ++i) {
        auto* joint = fr_leg->Add();
        joint->set_position(0.1 + i * 0.3);
        joint->set_velocity(0.2 + i * 0.3);
        joint->set_torque(0.3 + i * 0.3);
        joint->set_temperature(40.0 + i);
    }

    auto* hl_leg = observation.mutable_joint_data()->mutable_hl_leg();
    for (int i = 0; i < 3; ++i) {
        auto* joint = hl_leg->Add();
        joint->set_position(0.1 + i * 0.3);
        joint->set_velocity(0.2 + i * 0.3);
        joint->set_torque(0.3 + i * 0.3);
        joint->set_temperature(40.0 + i);
    }

    auto* hr_leg = observation.mutable_joint_data()->mutable_hr_leg();
    for (int i = 0; i < 3; ++i) {
        auto* joint = hr_leg->Add();
        joint->set_position(0.1 + i * 0.3);
        joint->set_velocity(0.2 + i * 0.3);
        joint->set_torque(0.3 + i * 0.3);
        joint->set_temperature(40.0 + i);
    }

    // Set contact force data
    auto* contact_force = observation.mutable_contact_force();
    for (int i = 0; i < 3; ++i) {
        contact_force->add_fl_leg(1.0 + i);
        contact_force->add_fr_leg(4.0 + i);
        contact_force->add_hl_leg(7.0 + i);
        contact_force->add_hr_leg(10.0 + i);
    }

    // Send the observation and receive the action
    Action action = client.GetAction(observation);

    // Print the received action
    std::cout << "Received action:" << std::endl;
    for (const auto& joint_cmd : action.robot_cmd().fl_leg()) {
        std::cout << "  Position: " << joint_cmd.position()
                  << ", Velocity: " << joint_cmd.velocity()
                  << ", Torque: " << joint_cmd.torque()
                  << ", Kp: " << joint_cmd.kp()
                  << ", Kd: " << joint_cmd.kd() << std::endl;
    }

    return 0;
}