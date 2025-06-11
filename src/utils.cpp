#include <iostream>
#include <fstream> 
#include "robot_types.h"
#include "realenv.grpc.pb.h"
#include "motion_spline.h"

const double kRadian2Degree = 180 / 3.1415926;

/// @brief Function to print all robot data.
/// @param robot_data Pointer to the RobotData structure containing the data to print.
void PrintRobotData(const RobotData* robot_data, std::ofstream& file) {
    if (!robot_data) {
        std::cerr << "Invalid robot data pointer!" << std::endl;
        return;
    }

    if (!file.is_open()) {
        std::cerr << "File stream is not open!" << std::endl;
        return;
    }

    // Print tick (timestamp)
    file << "Tick: " << robot_data->tick << std::endl;

    // Print IMU data
    file << "IMU Data:" << std::endl;
    file << "  Roll Angle: " << robot_data->imu.angle_roll << " degrees" << std::endl;
    file << "  Pitch Angle: " << robot_data->imu.angle_pitch << " degrees" << std::endl;
    file << "  Yaw Angle: " << robot_data->imu.angle_yaw << " degrees" << std::endl;
    file << "  Roll Angular Velocity: " << robot_data->imu.angular_velocity_roll << " deg/s" << std::endl;
    file << "  Pitch Angular Velocity: " << robot_data->imu.angular_velocity_pitch << " deg/s" << std::endl;
    file << "  Yaw Angular Velocity: " << robot_data->imu.angular_velocity_yaw << " deg/s" << std::endl;
    file << "  Acceleration X: " << robot_data->imu.acc_x << " m/s^2" << std::endl;
    file << "  Acceleration Y: " << robot_data->imu.acc_y << " m/s^2" << std::endl;
    file << "  Acceleration Z: " << robot_data->imu.acc_z << " m/s^2" << std::endl;

    // Print joint data
    file << "Joint Data:" << std::endl;
    const char* leg_names[] = {"Front Left Leg", "Front Right Leg", "Hind Left Leg", "Hind Right Leg"};
    const JointData* legs[] = {robot_data->joint_data.fl_leg, robot_data->joint_data.fr_leg,
                               robot_data->joint_data.hl_leg, robot_data->joint_data.hr_leg};

    for (int leg = 0; leg < 4; ++leg) {
        file << "  " << leg_names[leg] << ":" << std::endl;
        for (int joint = 0; joint < 3; ++joint) {
            const JointData& joint_data = legs[leg][joint];
            file << "    Joint " << joint + 1 << ":" << std::endl;
            file << "      Position: " << joint_data.position * kRadian2Degree << " degrees" << std::endl;
            file << "      Velocity: " << joint_data.velocity * kRadian2Degree << " degrees/s" << std::endl;
            file << "      Torque: " << joint_data.torque << " Nm" << std::endl;
            file << "      Temperature: " << joint_data.temperature << " °C" << std::endl;
        }
    }

    // Print contact force data
    file << "Contact Force Data:" << std::endl;
    for (int leg = 0; leg < 4; ++leg) {
        file << "  " << leg_names[leg] << " Forces (x, y, z): ";
        file << robot_data->contact_force.leg_force[leg * 3 + 0] << ", "
                  << robot_data->contact_force.leg_force[leg * 3 + 1] << ", "
                  << robot_data->contact_force.leg_force[leg * 3 + 2] << std::endl;
    }

}

/// @brief Converts RobotData into an Observation object.
/// @param robot_data The RobotData structure to convert.
/// @return An Observation object populated with data from RobotData.
realenv::Observation ConvertRobotDataToObservation(const RobotData& robot_data) {
    realenv::Observation observation;

    // Set the tick (timestamp)
    observation.set_tick(robot_data.tick);

    // Set IMU data
    auto* imu = observation.mutable_imu();
    imu->set_angle_roll(robot_data.imu.angle_roll);
    imu->set_angle_pitch(robot_data.imu.angle_pitch);
    imu->set_angle_yaw(robot_data.imu.angle_yaw);
    imu->set_angular_velocity_roll(robot_data.imu.angular_velocity_roll);
    imu->set_angular_velocity_pitch(robot_data.imu.angular_velocity_pitch);
    imu->set_angular_velocity_yaw(robot_data.imu.angular_velocity_yaw);
    imu->set_acc_x(robot_data.imu.acc_x);
    imu->set_acc_y(robot_data.imu.acc_y);
    imu->set_acc_z(robot_data.imu.acc_z);

    // Set joint data for each leg
    auto* joint_data = observation.mutable_joint_data();
    auto* fl_leg = joint_data->mutable_fl_leg();
    auto* fr_leg = joint_data->mutable_fr_leg();
    auto* hl_leg = joint_data->mutable_hl_leg();
    auto* hr_leg = joint_data->mutable_hr_leg();

    for (int i = 0; i < 3; ++i) {
        // Front Left Leg
        auto* fl_joint = fl_leg->Add();
        fl_joint->set_position(robot_data.joint_data.fl_leg[i].position);
        fl_joint->set_velocity(robot_data.joint_data.fl_leg[i].velocity);
        fl_joint->set_torque(robot_data.joint_data.fl_leg[i].torque);
        fl_joint->set_temperature(robot_data.joint_data.fl_leg[i].temperature);

        // Front Right Leg
        auto* fr_joint = fr_leg->Add();
        fr_joint->set_position(robot_data.joint_data.fr_leg[i].position);
        fr_joint->set_velocity(robot_data.joint_data.fr_leg[i].velocity);
        fr_joint->set_torque(robot_data.joint_data.fr_leg[i].torque);
        fr_joint->set_temperature(robot_data.joint_data.fr_leg[i].temperature);

        // Hind Left Leg
        auto* hl_joint = hl_leg->Add();
        hl_joint->set_position(robot_data.joint_data.hl_leg[i].position);
        hl_joint->set_velocity(robot_data.joint_data.hl_leg[i].velocity);
        hl_joint->set_torque(robot_data.joint_data.hl_leg[i].torque);
        hl_joint->set_temperature(robot_data.joint_data.hl_leg[i].temperature);

        // Hind Right Leg
        auto* hr_joint = hr_leg->Add();
        hr_joint->set_position(robot_data.joint_data.hr_leg[i].position);
        hr_joint->set_velocity(robot_data.joint_data.hr_leg[i].velocity);
        hr_joint->set_torque(robot_data.joint_data.hr_leg[i].torque);
        hr_joint->set_temperature(robot_data.joint_data.hr_leg[i].temperature);
    }

    // Set contact force data
    auto* contact_force = observation.mutable_contact_force();
    for (int i = 0; i < 3; ++i) {
        contact_force->add_fl_leg(robot_data.contact_force.fl_leg[i]);
        contact_force->add_fr_leg(robot_data.contact_force.fr_leg[i]);
        contact_force->add_hl_leg(robot_data.contact_force.hl_leg[i]);
        contact_force->add_hr_leg(robot_data.contact_force.hr_leg[i]);
    }

    return observation;
}


#include "robot_types.h"
#include "realenv.grpc.pb.h"


/// @brief Creates a RobotCmd structure from a set of leg positions.
/// @param fl_leg_positions The positions of the front left leg.
/// @param fr_leg_positions The positions of the front right leg.
/// @param hl_leg_positions The positions of the hind left leg.
/// @param hr_leg_positions The positions of the hind right leg.
/// @return A RobotCmd structure populated with the given leg positions.
RobotCmd CreateRobotCmdFromNumber(double fl_leg_positions[3], double fr_leg_positions[3], double hl_leg_positions[3], double hr_leg_positions[3], double kp, double kd) {
    RobotCmd robot_cmd;
    memset(&robot_cmd, 0, sizeof(robot_cmd));

    for (int i = 0; i < 3; i++) {
        robot_cmd.fl_leg[i].position = fl_leg_positions[i];
        robot_cmd.fl_leg[i].velocity = 0;
        robot_cmd.fl_leg[i].torque = 0;
        robot_cmd.fl_leg[i].kp = kp;
        robot_cmd.fl_leg[i].kd = kd;
    }

    for (int i = 0; i < 3; i++) {
        robot_cmd.fr_leg[i].position = fr_leg_positions[i];
        robot_cmd.fr_leg[i].velocity = 0;
        robot_cmd.fr_leg[i].torque = 0;
        robot_cmd.fr_leg[i].kp = kp;
        robot_cmd.fr_leg[i].kd = kd;
    }

    for (int i = 0; i < 3; i++) {
        robot_cmd.hl_leg[i].position = hl_leg_positions[i];
        robot_cmd.hl_leg[i].velocity = 0;
        robot_cmd.hl_leg[i].torque = 0;
        robot_cmd.hl_leg[i].kp = kp;
        robot_cmd.hl_leg[i].kd = kd;
    }

    for (int i = 0; i < 3; i++) {
        robot_cmd.hr_leg[i].position = hr_leg_positions[i];
        robot_cmd.hr_leg[i].velocity = 0;
        robot_cmd.hr_leg[i].torque = 0;
        robot_cmd.hr_leg[i].kp = kp;
        robot_cmd.hr_leg[i].kd = kd;
    }

    return robot_cmd;
}

/// @brief Converts a realenv::Action into a RobotCmd structure.
/// @param action The Action object received from the policy.
/// @return A RobotCmd structure populated with data from the Action.
RobotCmd CreateRobotCmd(const realenv::Action& action) {
    RobotCmd robot_cmd;

    // Extract joint commands for each leg
    const auto& fl_leg_cmds = action.robot_cmd().fl_leg();
    const auto& fr_leg_cmds = action.robot_cmd().fr_leg();
    const auto& hl_leg_cmds = action.robot_cmd().hl_leg();
    const auto& hr_leg_cmds = action.robot_cmd().hr_leg();

    // Helper lambda to copy joint commands
    auto copy_joint_cmds = [](const google::protobuf::RepeatedPtrField<realenv::Action::JointCmd>& joint_cmds, JointCmd* joint_data) {
        for (int i = 0; i < joint_cmds.size() && i < 3; ++i) {
            joint_data[i].position = joint_cmds[i].position();
            joint_data[i].velocity = joint_cmds[i].velocity();
            joint_data[i].torque = joint_cmds[i].torque();
            joint_data[i].kp = joint_cmds[i].kp();
            joint_data[i].kd = joint_cmds[i].kd();
        }
    };

    // Copy joint commands for each leg
    copy_joint_cmds(fl_leg_cmds, robot_cmd.fl_leg);
    copy_joint_cmds(fr_leg_cmds, robot_cmd.fr_leg);
    copy_joint_cmds(hl_leg_cmds, robot_cmd.hl_leg);
    copy_joint_cmds(hr_leg_cmds, robot_cmd.hr_leg);

    return robot_cmd;
}

/// @brief Prints the RobotCmd structure into a file.
/// @param robot_cmd The RobotCmd structure to print.
/// @param file The output file stream to write the data.
void PrintRobotCmd(const RobotCmd& robot_cmd, std::ofstream& file) {
    if (!file.is_open()) {
        std::cerr << "File stream is not open!" << std::endl;
        return;
    }

    const char* leg_names[] = {"Front Left Leg", "Front Right Leg", "Hind Left Leg", "Hind Right Leg"};
    const JointCmd* legs[] = {robot_cmd.fl_leg, robot_cmd.fr_leg, robot_cmd.hl_leg, robot_cmd.hr_leg};

    file << "Robot Command:" << std::endl;

    for (int leg = 0; leg < 4; ++leg) {
        file << "  " << leg_names[leg] << ":" << std::endl;
        for (int joint = 0; joint < 3; ++joint) {
            const JointCmd& joint_data = legs[leg][joint];
            file << "    Joint " << joint + 1 << ":" << std::endl;
            file << "      Position: " << joint_data.position * kRadian2Degree << " degrees" << std::endl;
            file << "      Velocity: " << joint_data.velocity * kRadian2Degree << " degrees/s" << std::endl;
            file << "      Torque: " << joint_data.torque << " Nm" << std::endl;
            file << "      Kp: " << joint_data.kp << std::endl;
            file << "      Kd: " << joint_data.kd << std::endl;
        }
    }
}

/// @brief Saves robot data to a CSV file.
/// @param robot_data Pointer to the RobotData structure containing the data to save.
/// @param file The output file stream to write the data.
void SaveRobotDataToCSV(const RobotData* robot_data, std::ofstream& file) {
    if (!robot_data || !file.is_open()) {
        std::cerr << "Invalid robot data pointer or file not open!" << std::endl;
        return;
    }

    // Write tick
    file << robot_data->tick << ",";

    // Write IMU data
    file << robot_data->imu.angle_roll << ","
         << robot_data->imu.angle_pitch << ","
         << robot_data->imu.angle_yaw << ","
         << robot_data->imu.angular_velocity_roll << ","
         << robot_data->imu.angular_velocity_pitch << ","
         << robot_data->imu.angular_velocity_yaw << ","
         << robot_data->imu.acc_x << ","
         << robot_data->imu.acc_y << ","
         << robot_data->imu.acc_z << ",";

    // Write joint data for each leg
    const JointData* legs[] = {robot_data->joint_data.fl_leg, 
                              robot_data->joint_data.fr_leg,
                              robot_data->joint_data.hl_leg, 
                              robot_data->joint_data.hr_leg};

    for (int leg = 0; leg < 4; ++leg) {
        for (int joint = 0; joint < 3; ++joint) {
            const JointData& joint_data = legs[leg][joint];
            file << joint_data.position * kRadian2Degree << ","
                 << joint_data.velocity * kRadian2Degree << ","
                 << joint_data.torque << ","
                 << joint_data.temperature << ",";
        }
    }

    // Write contact force data
    for (int leg = 0; leg < 4; ++leg) {
        file << robot_data->contact_force.leg_force[leg * 3 + 0] << ","
             << robot_data->contact_force.leg_force[leg * 3 + 1] << ","
             << robot_data->contact_force.leg_force[leg * 3 + 2] << ",";
    }

    file << std::endl;
}

/// @brief Writes CSV header to the file.
/// @param file The output file stream to write the header.
void WriteCSVHeader(std::ofstream& file) {
    if (!file.is_open()) {
        std::cerr << "File stream is not open!" << std::endl;
        return;
    }

    // Write header for tick
    file << "tick,";

    // Write header for IMU data
    file << "imu_roll,imu_pitch,imu_yaw,"
         << "imu_roll_vel,imu_pitch_vel,imu_yaw_vel,"
         << "imu_acc_x,imu_acc_y,imu_acc_z,";

    // Write header for joint data
    const char* leg_names[] = {"fl", "fr", "hl", "hr"};
    for (const char* leg : leg_names) {
        for (int joint = 1; joint <= 3; ++joint) {
            file << leg << "_j" << joint << "_pos,"
                 << leg << "_j" << joint << "_vel,"
                 << leg << "_j" << joint << "_torque,"
                 << leg << "_j" << joint << "_temp,";
        }
    }

    // Write header for contact forces
    for (const char* leg : leg_names) {
        file << leg << "_force_x," << leg << "_force_y," << leg << "_force_z,";
    }

    file << std::endl;
}