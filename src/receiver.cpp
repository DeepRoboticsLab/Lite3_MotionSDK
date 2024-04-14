/**
 * @file receiver.cpp
 * @author vcb (www.deeprobotics.cn)
 * @brief 
 * @version 0.1
 * @date 2023-03-17
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "receiver.h"
#include "common/command.h"

using namespace std;
using namespace lite3;

Receiver::Receiver(int port){
  local_port = port;
  StartWork();
}

Receiver::~Receiver(){

}
void Receiver::Work()
{
  // EthCommand c;
  CommandMessage cm;
  // Command cmd_test;
  UDPServer udpServer;
  timespec test_time;

  udpServer.onRawMessageReceived = [&](const char* message, int length, string ipv4, uint16_t port) {
    clock_gettime(1,&test_time);
    memcpy(&cm,message,sizeof(cm));
    Command nc(cm.command.code,cm.command.paramters_size, cm.data_buffer);
    if(cm.command.type == command_type::CommandType::kMessValues){
      switch (cm.command.code){
        case ROBOT_STATE_CMD:
          clock_gettime(1,&test_time);
          memcpy(&state_rec_, cm.data_buffer, sizeof(state_rec_));
           if (CallBack_) CallBack_(ROBOT_STATE_CMD);
          break;
      default:
        break;
      }
    }
  };
    // Bind the server to a port.
    udpServer.Bind(local_port, [](int errorCode, string errorMessage) {
    // BINDING FAILED:
    cout << errorCode << " : " << errorMessage << endl;
  });
	while (1){
    sleep(1);
	}
}

void Receiver::StartWork()
{
  std::thread work_thread(std::bind(&Receiver::Work, this));
	work_thread.detach();
}
RobotData& Receiver::GetState(){
  return state_rec_;
}
