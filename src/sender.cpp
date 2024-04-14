/**
 * @file sender.cpp
 * @author vcb (www.deeprobotics.cn)
 * @brief 
 * @version 0.1
 * @date 2023-03-17
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "sender.h"
using namespace std;
using namespace lite3;
Sender::Sender(){
  const string IP = "192.168.1.120";
  const uint16_t PORT = 43897;
  udp_socket_ = new UDPSocket(true);
  udp_socket_->Connect(IP, PORT);
}

Sender::Sender(string ip , uint16_t port){
  const string IP = ip;
  const uint16_t PORT = port;
  udp_socket_ = new UDPSocket(true);
  udp_socket_->Connect(IP, PORT);
}

Sender::~Sender(){

}

void Sender::CmdDone(Command& command_temp){
  CommandMessage command_message={{0}};
  uint32_t message_size = 0;
    command_message.command.code = command_temp.GetCommandCode().to_ulong();
      
      if(command_temp.GetCommandParameters()==0){
        // 普通命令
        command_message.command.type = command_type::CommandType::kSingleValue;
        command_message.command.value = command_temp.GetCommandValue();
        message_size = sizeof(command_message.command);
      }
      else{
        // 带大量数据的命令
        command_message.command.type = command_type::CommandType::kMessValues;
        command_message.command.paramters_size = command_temp.GetCommandParametersSize();
        if(command_message.command.paramters_size > kCommandDataBufferSize){
          std::cout <<"[Error E_Speaker] The message of over load !"<< std::endl;
          return;
        }
        // std::cout << "send   " << count++ <<std:: endl;
      }
        // 复制数据并释放Command中的内容
        memcpy(command_message.data_buffer,
              command_temp.GetCommandParameters(),
              command_message.command.paramters_size);
        delete [](char*)command_temp.GetCommandParameters();
        
        message_size = sizeof(command_message.command) +
                                      command_message.command.paramters_size;
        char buffer[4096];
        memcpy(buffer,&command_message,message_size);                              
        udp_socket_->Send(buffer,message_size);
}

void Sender::SendCmd(RobotCmd& cmd){
  size_t cmd_size = sizeof(cmd);
  char *buffer = new char[cmd_size];
  memcpy(buffer, &cmd, cmd_size);
  Command command_temp(0x0111,sizeof(cmd), buffer);
  CmdDone(command_temp);
}

void Sender::AllJointBackZero(){
  Command command_temp(0x31010C05,0, 0);
  CmdDone(command_temp);
}

void Sender::RobotStateInit(){
  Command command_temp(0x31010C05,0, 0);
  CmdDone(command_temp);
  usleep(1000*1000*7);
  Command command_temp_1(0x0114,0,0);
  CmdDone(command_temp_1);//PS：超过5ms，未发数据set_send(cmd)，会失去控制权，要重新发送获取控制权
  CmdDone(command_temp_1);//PS：超过5ms，未发数据set_send(cmd)，会失去控制权，要重新发送获取控制权
}

void Sender::SetCmd(uint32_t code , uint32_t value){
  Command command_temp(code, value);
  CmdDone(command_temp);
}

void Sender::ControlGet(uint32_t mode){
  if(mode == ROBOT){
    RobotCmd cmd;
    for(int i = 0; i < 12; i++){
      cmd.joint_cmd[i].position = 0.0;
      cmd.joint_cmd[i].torque = 0.0;
      cmd.joint_cmd[i].velocity = 0.0;
      cmd.joint_cmd[i].kp = 0.0;
      cmd.joint_cmd[i].kd = 5.0;
    }
    SendCmd(cmd);
    sleep(2);
    Command command_temp(0x0113,0, 0);
    CmdDone(command_temp);
  }else if (mode == SDK){
    Command command_temp(0x0114,0, 0);
    CmdDone(command_temp);
  }
}
