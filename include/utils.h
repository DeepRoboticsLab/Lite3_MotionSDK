#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include "robot_types.h"
#include "realenv.grpc.pb.h"



/// @brief Function to print all robot data.
/// @param robot_data Pointer to the RobotData structure containing the data to print.
void PrintRobotData(const RobotData* robot_data, std::ofstream& file);

/// @brief Converts RobotData into an Observation object.
/// @param robot_data The RobotData structure to convert.
/// @return An Observation object populated with data from RobotData.
realenv::Observation ConvertRobotDataToObservation(const RobotData& robot_data);


/// @brief Converts a realenv::Action into a RobotCmd structure.
/// @param action The Action object received from the policy.
/// @return A RobotCmd structure populated with data from the Action.
RobotCmd CreateRobotCmd(const realenv::Action& action);

/// @brief Prints the RobotCmd structure into a file.
/// @param robot_cmd The RobotCmd structure to print.
/// @param file The output file stream to write the data.
void PrintRobotCmd(const RobotCmd& robot_cmd, std::ofstream& file);


#endif // UTILS_H