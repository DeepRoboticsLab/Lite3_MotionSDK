/// @file lite3_types.h
/// @author vcb (www.deeprobotics.cn)
/// @brief 
/// @version 0.1
/// @date 2022-09-25
/// @copyright Copyright (c) 2023
 


#ifndef LITE3_TYPES_H_
#define LITE3_TYPES_H_

#include <stdint.h>
#include <array>

namespace lite3{
/// @brief A struct used to store the data collected from an IMU sensor.
struct ImuData {
  int32_t timestamp; // Timestamp of the data in milliseconds.
  union {
    float buffer_float[9]; // An array containing 9 float elements.
    uint8_t buffer_byte[3][12]; // A 2D array with 3 rows and 12 columns of uint8_t elements used to store raw data.
    struct {
      float roll; // Roll angle (unit: degrees).
      float pitch; // Pitch angle (unit: degrees).
      float yaw; // Yaw angle (unit: degrees).

      float omega_x; // x-axis angular velocity (unit: degrees/second).
      float omega_y; // y-axis angular velocity (unit: degrees/second).
      float omega_z; // z-axis angular velocity (unit: degrees/second).

      float acc_x; // X-axis acceleration (unit: m/s^2).
      float acc_y; // Y-axis acceleration (unit: m/s^2).
      float acc_z; // Z-axis acceleration (unit: m/s^2).
    }; // An anonymous structure used to access the buffer member as float.
  }; // An anonymous union used to provide different ways of accessing the data.
};

/// @brief Struct containing data related to a motor.
typedef struct {
  float position; // Motor position (unit: radians).
  float velocity; // Motor velocity (unit: radians/second).
  float torque; // Motor torque (unit: Nm). */
  float temperature; // Motor temperature (unit: Celsius).
} JointData;

/// @brief Struct containing data related to a robot.
typedef struct {
  union {
    std::array<JointData, 12> joint_data; // Joint data for all 12 motors.
    struct {
      std::array<JointData, 3> fl_leg; // Joint data for the front left leg motors.
      std::array<JointData, 3> fr_leg; // Joint data for the front right leg motors.
      std::array<JointData, 3> hl_leg; // Joint data for the hind left leg motors.
      std::array<JointData, 3> hr_leg; // Joint data for the hind right leg motors.
    };
  };
} LegData;

/// @brief Struct containing command data for a joint.
typedef struct {
  float position; // Desired joint position (unit: radians).
  float velocity; // Desired joint velocity (unit: radians/second).
  float torque; // Desired joint torque (unit: Nm).
  float kp; // Proportional gain of joint controller.
  float kd; // Derivative gain of joint controller.
} JointCmd;

/// @brief Struct containing command data for a robot.
typedef struct {
  union {
    std::array<JointCmd, 12> joint_cmd;  // Joint commands for all 12 joints.
    struct {
      std::array<JointCmd, 3> fl_leg; // Joint commands for the front left leg joints.
      std::array<JointCmd, 3> fr_leg; // Joint commands for the front right leg joints.
      std::array<JointCmd, 3> hl_leg; // Joint commands for the hind left leg joints.
      std::array<JointCmd, 3> hr_leg; // Joint commands for the hind left leg joints.
    };
  };
} RobotCmd;

/// @brief Struct containing contact force data for a robot.
/// @description The `leg_force` array contains the forces for all 12 motors, while the
/// individual leg force arrays (`fl_leg`, `fr_leg`, `hl_leg`, `hr_leg`) contain the forces for
/// the specific legs of the robot. The forces are given in x, y, and z direction.
///
/// @note In most cases, only the z-direction force is used.
typedef struct {
  union {
    std::array<double, 12> leg_force; // Array of leg forces for all 12 motors.
    struct {
      std::array<double, 3> fl_leg; // Forces for the front left leg motors (in x, y, and z direction).
      std::array<double, 3> fr_leg; // Forces for the front right leg motors (in x, y, and z direction).
      std::array<double, 3> hl_leg; // Forces for the hind left leg motors (in x, y, and z direction).
      std::array<double, 3> hr_leg; // Forces for the hind right leg motors (in x, y, and z direction).
    };
  };
} ContactForce;


/// @brief 
/// Struct containing data for a robot.
/// The tick field is a timestamp for the data. The imu, joint_data, and contact_force
/// fields contain the IMU data, joint data, and contact force data for the robot, respectively.
typedef struct{
  uint32_t tick;              // Timestamp for the data.
  ImuData imu;                // IMU data for the robot.
  LegData joint_data;         // Joint data for the robot.
  ContactForce contact_force; // Contact force data for the robot.
}RobotData;

}

#endif  ///< LITE3_TYPES_H_