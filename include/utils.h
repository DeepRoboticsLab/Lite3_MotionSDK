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

/// @brief Creates a RobotCmd structure from a set of leg positions.
/// @param fl_leg_positions The positions of the front left leg.
/// @param fr_leg_positions The positions of the front right leg.
/// @param hl_leg_positions The positions of the hind left leg.
/// @param hr_leg_positions The positions of the hind right leg.
/// @return A RobotCmd structure populated with the given leg positions.
RobotCmd CreateRobotCmdFromNumber(double fl_leg_positions[3], double fr_leg_positions[3], double hl_leg_positions[3], double hr_leg_positions[3], double kp, double kd);


/// @brief Converts a realenv::Action into a RobotCmd structure.
/// @param action The Action object received from the policy.
/// @return A RobotCmd structure populated with data from the Action.
RobotCmd CreateRobotCmd(const realenv::Action& action);

/// @brief Prints the RobotCmd structure into a file.
/// @param robot_cmd The RobotCmd structure to print.
/// @param file The output file stream to write the data.
void PrintRobotCmd(const RobotCmd& robot_cmd, std::ofstream& file);

/// @brief Saves robot data to a CSV file.
/// @param robot_data Pointer to the RobotData structure containing the data to save.
/// @param file The output file stream to write the data.
void SaveRobotDataToCSV(const RobotData* robot_data, std::ofstream& file);

/// @brief Writes the CSV header to the file.
/// @param file The output file stream to write the header.
void WriteCSVHeader(std::ofstream& file);


#endif // UTILS_H