# -*- coding: UTF-8 -*-
import sys
sys.path.append('./lib')
import deeprobotics_lite3_motion_sdk_py as dr
from motion_example import MotionExample


is_message_updated_ = False

def handler(code):
    if(code == 0x0906):
      is_message_updated_ = True
    

if __name__ == "__main__" :
  timer = dr.DRTimer()
  now_time = 0.0
  start_time = 0.0

  sender = dr.Sender('192.168.1.120', 43893)
  receiver = dr.Receiver(43897)
  receiver.RegisterCallBack(handler)
  robot_data = receiver.GetState()
  receiver.StartWork()
  timer.TimeInit(1)
  sender.RobotStateInit()
  
  start_time = timer.GetCurrentTime()
  
  robot_set_up_demo = MotionExample()
  robot_set_up_demo.GetInitData(receiver.GetState().joint_data, 0.000)
  
  time_tick = 0
  robot_joint_cmd = dr.RobotCmd()
  
  while True:
    if timer.TimerInterrupt() == True:
      continue
    now_time = timer.GetIntervalTime(start_time)
    time_tick = time_tick+1

    if time_tick < 1000:
      robot_set_up_demo.PreStandUp(robot_joint_cmd, now_time, receiver.GetState())
    if time_tick == 1000:
      robot_set_up_demo.GetInitData(receiver.GetState().joint_data, now_time)
    if time_tick >= 1000:
      robot_set_up_demo.StandUp(robot_joint_cmd, now_time, receiver.GetState())
    if time_tick >= 10000:
      sender.ControlGet(1)
      break
    if is_message_updated_:
      pass
    sender.SendCmd(robot_joint_cmd)
