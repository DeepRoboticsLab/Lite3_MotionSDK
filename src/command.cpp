/**
 * @file command.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-09-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "common/command.h"

using namespace lite3;

Command::Command() {
  command_code_ = 0;
  command_value_ = 0;
  command_parameters_ = NULL;
}

Command::Command(uint32_t command_code,int32_t command_value){
  command_code_ = command_code;
  command_value_ = command_value;
  command_parameters_ = NULL;
}

Command::Command(uint32_t command_code,
                 size_t command_parameters_size,
                 void* command_parameters){
  command_code_ = command_code;
  command_parameters_size_ = command_parameters_size;
  command_parameters_ = command_parameters;
}

Command::~Command() {
}


std::bitset<32>& Command::GetCommandCode(){
  return command_code_;
}


int32_t Command::GetCommandValue(){
  return command_value_;
}


size_t Command::GetCommandParametersSize(){
  return command_parameters_size_;
}

// 这个返回的参数，必须由使用者释放掉
// 因为底层无法知道它的类型，而无法自动释放。
void* Command::GetCommandParameters(){
  return command_parameters_;
}

