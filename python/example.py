# -*- coding: UTF-8 -*-
import sys
import time
sys.path.append('./lib')
import deeprobotics_lite3_motion_sdk_py as dr
from motion_example import MotionExample



is_message_updated = False
def handler(code):
  global is_message_updated
  if(code == 0x0906):
    is_message_updated = True
      
def print_robot_data(robot_data):
  print("joint_pos: ")
  for i in range(12):
    print(round(robot_data.joint_data.joint_data[i].position, 3), end=" ")
  print("\nimu_acc: ")
  print(round(robot_data.imu.acc_x, 3),\
        round(robot_data.imu.acc_y, 3),\
        round(robot_data.imu.acc_z, 3))

timer = dr.DRTimer()

sender = dr.Sender('192.168.1.120', 43893)
receiver = dr.Receiver(43897)
receiver.RegisterCallBack(handler)
robot_data = receiver.GetState()
receiver.StartWork()
timer.TimeInit(1)
sender.RobotStateInit()
robot_set_up_demo = MotionExample()
time_tick = 0
robot_joint_cmd = dr.RobotCmd()

while not is_message_updated or time_tick < 1000 :#make sure received data is correct
  start_time = timer.GetCurrentTime()
  robot_data = receiver.GetState()
  robot_set_up_demo.GetInitData(robot_data.joint_data, 0) 
  time_tick += 1
  time.sleep(0.001)  
  if time_tick%1000==0:
    print_robot_data(robot_data)
  
  
time_tick = 0
while is_message_updated:
  now_time = timer.GetIntervalTime(start_time)
  robot_data = receiver.GetState()
  if timer.TimerInterrupt() == True:
    continue

  time_tick = time_tick+1
  if time_tick < 2000:
    robot_set_up_demo.PreStandUp(robot_joint_cmd, now_time, robot_data)
  if time_tick == 2000:
    robot_set_up_demo.GetInitData(robot_data.joint_data, now_time)
  if time_tick >= 2000:
    robot_set_up_demo.StandUp(robot_joint_cmd, now_time, robot_data)
  if time_tick >= 10000:
    sender.ControlGet(1)
    break
  if is_message_updated == True:
    print_robot_data(robot_data)
    # sender.SendCmd(robot_joint_cmd)
    continue
