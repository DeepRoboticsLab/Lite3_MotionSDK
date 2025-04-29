/// @file main.cpp
/// @author your name (you@domain.com)
/// @brief 
/// @version 0.1
/// @date 2022-09-13 
/// @copyright Copyright (c) 2022


#include "udpsocket.hpp"
#include "udpserver.hpp"
#include "sender.h"
#include "dr_timer.h"
#include "receiver.h"
#include "testexample.h"
#include "utils.h"
#include "grpc_client.h"
#include <iostream>
#include <time.h>
#include <string.h>
#include <fstream> 

using namespace std;

  bool is_message_updated_ = false; ///< Flag to check if message has been updated
  /**
   * @brief Callback function to set message update flag
   * 
   * @param code The code indicating the type of message received
   */
  void OnMessageUpdate(uint32_t code){
    if(code == 0x0906){
      is_message_updated_ = true;
    }
  }

void GenerateTestRobotCmd(RobotCmd &robot_joint_cmd){
  for (int i = 0; i < 12; i++) {
    robot_joint_cmd.joint_cmd[i].torque = 0;
    robot_joint_cmd.joint_cmd[i].kp = 10;
    robot_joint_cmd.joint_cmd[i].kd = 0.7;
  }

  robot_joint_cmd.fl_leg[0].position = 0;
  robot_joint_cmd.fl_leg[0+1].position = -40 * kDegree2Radian;
  robot_joint_cmd.fl_leg[0+2].position = 150 * kDegree2Radian;
  robot_joint_cmd.fl_leg[0].velocity = 0;
  robot_joint_cmd.fl_leg[0+1].velocity = 0;
  robot_joint_cmd.fl_leg[0+2].velocity = 0;


  robot_joint_cmd.fr_leg[0].position = 0 * kDegree2Radian;
  robot_joint_cmd.fr_leg[0+1].position = -40 * kDegree2Radian;
  robot_joint_cmd.fr_leg[0+2].position = 150 * kDegree2Radian;
  robot_joint_cmd.fr_leg[0].velocity = 0;
  robot_joint_cmd.fr_leg[0+1].velocity = 0;
  robot_joint_cmd.fr_leg[0+2].velocity = 0;


  robot_joint_cmd.hl_leg[0].position = 0 * kDegree2Radian;
  robot_joint_cmd.hl_leg[0+1].position = -40 * kDegree2Radian;
  robot_joint_cmd.hl_leg[0+2].position = 150 * kDegree2Radian;
  robot_joint_cmd.hl_leg[0].velocity = 0;
  robot_joint_cmd.hl_leg[0+1].velocity = 0;
  robot_joint_cmd.hl_leg[0+2].velocity = 0;

  robot_joint_cmd.hr_leg[0].position = 0 * kDegree2Radian;
  robot_joint_cmd.hr_leg[0+1].position = -40 * kDegree2Radian;
  robot_joint_cmd.hr_leg[0+2].position = 150 * kDegree2Radian;
  robot_joint_cmd.hr_leg[0].velocity = 0;
  robot_joint_cmd.hr_leg[0+1].velocity = 0;
  robot_joint_cmd.hr_leg[0+2].velocity = 0;

}

int main(int argc, char* argv[]){
  // Create a client connected to the server
  PolicyServiceClient client(grpc::CreateChannel("localhost:50051", grpc::InsecureChannelCredentials()));

  DRTimer set_timer;
  double now_time,start_time;
  RobotCmd robot_joint_cmd;
  memset(&robot_joint_cmd, 0, sizeof(robot_joint_cmd));

  Sender* send_cmd          = new Sender("192.168.2.1",43893);              ///< Create send thread
  Receiver* robot_data_recv = new Receiver();                                 ///< Create a receive resolution
  robot_data_recv->RegisterCallBack(OnMessageUpdate);
  TestExample test_demo;                                            ///< Demos for testing can be deleted by yourself
  RobotData *robot_data = &robot_data_recv->GetState();

  robot_data_recv->StartWork();
  set_timer.TimeInit(5);                                                      ///< Timer initialization, input: cycle; Unit: ms
  send_cmd->RobotStateInit();                                                 ///< Return all joints to zero and gain control

  start_time = set_timer.GetCurrentTime();                                    ///< Obtain time for algorithm usage
  test_demo.GetInitData(robot_data->joint_data,0.000);                ///< Obtain all joint states once before each stage (action)
  
  int time_tick = 0;
  while(1){
    if (set_timer.TimerInterrupt() == true){                                  ///< Time interrupt flag
      continue;
    }
    now_time = set_timer.GetIntervalTime(start_time);                         ///< Get the current time
    time_tick++;
    // if(time_tick < 1000){
    //   robot_set_up_demo.PreStandUp(robot_joint_cmd,now_time,*robot_data);     ///< Stand up and prepare for action
    // } 
    // if(time_tick == 1000){
    //   robot_set_up_demo.GetInitData(robot_data->joint_data,now_time);         ///< Obtain all joint states once before each stage (action)
    // }
    // if(time_tick >= 1000 ){
    //   robot_set_up_demo.StandUp(robot_joint_cmd,now_time,*robot_data);        ///< Full stand up
    // }
    // if (time_tick < 10000) {
    //   GenerateTestRobotCmd(robot_joint_cmd);  ///< Generate test robot command
    // }
    if (time_tick < 10000) {
      float fl_leg_positions[3] = {-28, -200, 120};  
      float fr_leg_positions[3] = {0, -200, 34};
      float hl_leg_positions[3] = {-28, -200, 120};
      float hr_leg_positions[3] = {0, -200, 120};
      robot_joint_cmd = CreateRobotCmdFromNumber(fl_leg_positions, fr_leg_positions, hl_leg_positions, hr_leg_positions);

      test_demo.Motion(robot_joint_cmd,now_time,*robot_data);
    }
    // if (time_tick == 1000) {
    //   test_demo.GetInitData(robot_data->joint_data,now_time);
    // }
    // if (time_tick > 1000) {
    //   float fl_leg_positions[3] = {0, 20, 80};  
    //   float fr_leg_positions[3] = {20, 20, 80};
    //   float hl_leg_positions[3] = {0, -40, 80};
    //   float hr_leg_positions[3] = {0, -40, 80};
    //   robot_joint_cmd = CreateRobotCmdFromNumber(fl_leg_positions, fr_leg_positions, hl_leg_positions, hr_leg_positions);

    //   test_demo.Motion(robot_joint_cmd,now_time,*robot_data);  
    // }
    if(time_tick >= 10000){
      send_cmd->ControlGet(ROBOT);                                            ///< Return the control right, input: ROBOT: Original algorithm control of the robot .  SDK: SDK control PS: over 50ms, no data set sent_ Send (cmd), you will lose control, you need to resend to obtain control
      break;
    }
    if(is_message_updated_){ 
      // send_cmd->SendCmd(robot_joint_cmd);  
    }  

    
    
    // print robot data
    // Open the file in append mode
    std::ofstream file("robot_data.txt", std::ios::app);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: robot_data.txt" << std::endl;
        return 1; // Exit the program if the file cannot be opened
    }
    // Print robot data to the file
    PrintRobotData(robot_data, file);
    // Close the file


    // // Convert RobotData to Observation
    // realenv::Observation observation = ConvertRobotDataToObservation(*robot_data);
    // // Send the observation and receive the action
    // realenv::Action action = client.GetAction(observation);
    // // Convert the action back to RobotCmd
    // robot_joint_cmd = CreateRobotCmd(action);

    // print robot data
    // Open the file in append mode
    if (!file.is_open()) {
        std::cerr << "Failed to open file: robot_data.txt" << std::endl;
        return 1; // Exit the program if the file cannot be opened
    }
    // Print robot data to the file
    PrintRobotCmd(robot_joint_cmd, file);
    // Close the file
    file.close();

  }
  return 0;
} 
