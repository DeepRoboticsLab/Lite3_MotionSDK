# -*- coding: UTF-8 -*-
import sys
sys.path.append('./lib')

import numpy
import deeprobotics_lite3_motion_sdk_py as dr



kDegree2Radian = 3.1415926 / 180

goal_angle_fl = numpy.zeros(3) 
goal_angle_hl = numpy.zeros(3) 
goal_angle_fr = numpy.zeros(3)
goal_angle_hr = numpy.zeros(3)
init_angle_fl = numpy.zeros(3)
init_angle_fr = numpy.zeros(3)
init_angle_hl = numpy.zeros(3)
init_angle_hr = numpy.zeros(3)

class MotionExample:
  def __init__(self):
    self.init_time = 0.0
  
  def CubicSpline(self, 
                  init_position, init_velocity,
                  goal_position, goal_velocity,
                  run_time, cycle_time, total_time):
    a=0.0
    b=0.0
    c = 0.0
    d = 0.0    
    d = init_position
    c = init_velocity
    a = (goal_velocity * total_time - 2 * goal_position + init_velocity * total_time + 
        2 * init_position) /pow(total_time, 3)
    b = (3 * goal_position - goal_velocity * total_time - 2 * init_velocity * total_time - 
        3 * init_position) / pow(total_time, 2)

    if run_time > total_time:
      run_time = total_time
    sub_goal_position = a * pow(run_time, 3) + b * pow(run_time, 2) + c * run_time + d

    if run_time + cycle_time > total_time:
      run_time = total_time - cycle_time
    sub_goal_position_next = a * pow(run_time + cycle_time, 3) + \
                             b * pow(run_time + cycle_time, 2) + \
                             c * (run_time + cycle_time) + d

    if run_time + cycle_time * 2 > total_time:
      run_time = total_time - cycle_time * 2
    sub_goal_position_next2 = a * pow(run_time + cycle_time * 2, 3) + \
                              b * pow(run_time + cycle_time * 2, 2) + \
                              c * (run_time + cycle_time * 2) + d
    return sub_goal_position, sub_goal_position_next, sub_goal_position_next2
  
  def SwingToAngle(self,
                   initial_angle, final_angle,
                   total_time, run_time,
                   cycle_time, side,
                   cmd, data):
    goal_angle = numpy.zeros(3)
    goal_angle_next = numpy.zeros(3)
    goal_angle_next2 = numpy.zeros(3)
    goal_velocity = numpy.zeros(3)
    leg_side = 0

    if side == "FL":
      leg_side = 0
    elif side == "FR":
      leg_side = 1
    elif side == "HL":
      leg_side = 2
    elif side == "HR":
      leg_side = 3
    else:
      print("Leg Side Error!!!")

    final_angle = final_angle
    
    for j in range(0, 3):
      goal_angle[j], goal_angle_next[j], goal_angle_next2[j] = \
        self.CubicSpline(initial_angle[j], 0, final_angle[j], 0, run_time,
                         cycle_time, total_time)
      goal_velocity[j] = (goal_angle_next[j] - goal_angle[j]) / cycle_time

    if True:
      cmd.joint_cmd[3 * leg_side].kp = 60
      cmd.joint_cmd[3 * leg_side + 1].kp = 60
      cmd.joint_cmd[3 * leg_side + 2].kp = 60
      cmd.joint_cmd[3 * leg_side].kd = 0.7
      cmd.joint_cmd[3 * leg_side + 1].kd = 0.7
      cmd.joint_cmd[3 * leg_side + 2].kd = 0.7
      cmd.joint_cmd[3 * leg_side].position = goal_angle[0]
      cmd.joint_cmd[3 * leg_side + 1].position = goal_angle[1]
      cmd.joint_cmd[3 * leg_side + 2].position = goal_angle[2]
      cmd.joint_cmd[3 * leg_side].velocity = goal_velocity[0]
      cmd.joint_cmd[3 * leg_side + 1].velocity = goal_velocity[1]
      cmd.joint_cmd[3 * leg_side + 2].velocity = goal_velocity[2]
      for i in range(0, 12):
        cmd.joint_cmd[i].torque = 0
    else:
      cmd.joint_cmd[3 * leg_side].kp = 0
      cmd.joint_cmd[3 * leg_side + 1].kp = 0
      cmd.joint_cmd[3 * leg_side + 2].kp = 0
      cmd.joint_cmd[3 * leg_side].kd = 0
      cmd.joint_cmd[3 * leg_side + 1].kd = 0
      cmd.joint_cmd[3 * leg_side + 2].kd = 0
      cmd.joint_cmd[3 * leg_side].position = 0
      cmd.joint_cmd[3 * leg_side + 1].position = 0
      cmd.joint_cmd[3 * leg_side + 2].position = 0
      cmd.joint_cmd[3 * leg_side].velocity = 0
      cmd.joint_cmd[3 * leg_side + 1].velocity = 0
      cmd.joint_cmd[3 * leg_side + 2].velocity = 0
      cmd.joint_cmd[3* leg_side].torque = 60 * (goal_angle[0] - data.joint_data.joint_data[3* leg_side].position) \
                                      + 0.7 * (goal_velocity[0] - data.joint_data.joint_data[3* leg_side].velocity)
      cmd.joint_cmd[3* leg_side+1].torque = 80 * (goal_angle[1] - data.joint_data.joint_data[3* leg_side+1].position) \
                                      + 0.7 * (goal_velocity[1] - data.joint_data.joint_data[3* leg_side+1].velocity)
      cmd.joint_cmd[3* leg_side+2].torque = 80 * (goal_angle[2] - data.joint_data.joint_data[3* leg_side+2].position) \
                                      + 0.7 * (goal_velocity[2] - data.joint_data.joint_data[3* leg_side+2].velocity)
    
  def PreStandUp(self, cmd, time, data_state):
    standup_time = 1.0
    cycle_time = 0.001
    goal_angle_fl = [0 * kDegree2Radian, -70 * kDegree2Radian, 150 * kDegree2Radian]
    goal_angle_fr = [0 * kDegree2Radian, -70 * kDegree2Radian, 150 * kDegree2Radian]
    goal_angle_hl = [0 * kDegree2Radian, -70 * kDegree2Radian, 150 * kDegree2Radian]
    goal_angle_hr = [0 * kDegree2Radian, -70 * kDegree2Radian, 150 * kDegree2Radian]

    if time <= self.init_time + standup_time:
      self.SwingToAngle(init_angle_fl, goal_angle_fl, standup_time, time - self.init_time,
                        cycle_time, "FL", cmd, data_state)
      self.SwingToAngle(init_angle_fr, goal_angle_fr, standup_time, time - self.init_time,
                        cycle_time, "FR", cmd, data_state)
      self.SwingToAngle(init_angle_hl, goal_angle_hl, standup_time, time - self.init_time,
                        cycle_time, "HL", cmd, data_state)
      self.SwingToAngle(init_angle_hr, goal_angle_hr, standup_time, time - self.init_time,
                        cycle_time, "HR", cmd, data_state)
   
  def StandUp(self, cmd, time, data_state):
    standup_time = 1.5
    cycle_time = 0.001
    goal_angle_fl = [0 * kDegree2Radian, -42 * kDegree2Radian, 78 * kDegree2Radian] 
    goal_angle_fr = [0 * kDegree2Radian, -42 * kDegree2Radian, 78 * kDegree2Radian]
    goal_angle_hl = [0 * kDegree2Radian, -42 * kDegree2Radian, 78 * kDegree2Radian]
    goal_angle_hr = [0 * kDegree2Radian, -42 * kDegree2Radian, 78 * kDegree2Radian]

    if (time <= self.init_time + standup_time):
      self.SwingToAngle(init_angle_fl, goal_angle_fl, standup_time, time - self.init_time,
                        cycle_time, "FL", cmd, data_state)
      self.SwingToAngle(init_angle_fr, goal_angle_fr, standup_time, time - self.init_time,
                        cycle_time, "FR", cmd, data_state)
      self.SwingToAngle(init_angle_hl, goal_angle_hl, standup_time, time - self.init_time,
                        cycle_time, "HL", cmd, data_state)
      self.SwingToAngle(init_angle_hr, goal_angle_hr, standup_time, time - self.init_time,
                        cycle_time, "HR", cmd, data_state)
    else:
      for i in range(0, 12):
        cmd.joint_cmd[i].torque = 0
        cmd.joint_cmd[i].kp = 80
        cmd.joint_cmd[i].kd = 0.7

      for i in range(0, 4):
        cmd.joint_cmd[3*i].position = 0
        cmd.joint_cmd[3*i+1].position = -42 * kDegree2Radian
        cmd.joint_cmd[3*i+2].position = 78 * kDegree2Radian
        cmd.joint_cmd[3*i].velocity = 0
        cmd.joint_cmd[3*i+1].velocity = 0
        cmd.joint_cmd[3*i+2].velocity = 0
        
  def GetInitData(self, data, time):
    self.init_time = time
    init_angle_fl[0] = data.joint_data[0].position
    init_angle_fl[1] = data.joint_data[1].position
    init_angle_fl[2] = data.joint_data[2].position

    init_angle_fr[0] = data.joint_data[3].position
    init_angle_fr[1] = data.joint_data[4].position
    init_angle_fr[2] = data.joint_data[5].position

    init_angle_hl[0] = data.joint_data[6].position
    init_angle_hl[1] = data.joint_data[7].position
    init_angle_hl[2] = data.joint_data[8].position

    init_angle_hr[0] = data.joint_data[9].position
    init_angle_hr[1] = data.joint_data[10].position
    init_angle_hr[2] = data.joint_data[11].position
