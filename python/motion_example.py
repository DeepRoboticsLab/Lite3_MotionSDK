# -*- coding: UTF-8 -*-
import sys
sys.path.append('./lib')

import numpy
import deeprobotics_lite3_motion_sdk_py as dr

kDegree2Radian = 3.1415926 / 180

class MotionExample:
  def __init__(self):
    self.init_time = 0.0
    self.init_angle_fl = numpy.zeros(3)
    self.init_angle_fr = numpy.zeros(3)
    self.init_angle_hl = numpy.zeros(3)
    self.init_angle_hr = numpy.zeros(3)
  
  def CubicSpline(self, x0, v0, xf, vf, run_time, cycle_time, total_time):
    d = x0
    c = v0
    a = (vf * total_time - 2 * xf + v0 * total_time + 
        2 * x0) /pow(total_time, 3)
    b = (3 * xf - vf * total_time - 2 * v0 * total_time - 
        3 * x0) / pow(total_time, 2)

    if run_time > total_time:
      run_time = total_time
    sub_xf = a * pow(run_time, 3) + b * pow(run_time, 2) + c * run_time + d

    if run_time + cycle_time > total_time:
      run_time = total_time
    sub_xf_next = a * pow(run_time + cycle_time, 3) + \
                             b * pow(run_time + cycle_time, 2) + \
                             c * (run_time + cycle_time) + d
    return sub_xf, sub_xf_next
  
  def SwingToAngle(self, q0, qf, total_time, run_time, cycle_time, side, cmd, data):
    goal_angle = numpy.zeros(3)
    goal_angle_next = numpy.zeros(3)
    goal_vel = numpy.zeros(3)
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
    
    for j in range(0, 3):
      goal_angle[j], goal_angle_next[j] = \
        self.CubicSpline(q0[j], 0, qf[j], 0, run_time,
                         cycle_time, total_time)
      goal_vel[j] = (goal_angle_next[j] - goal_angle[j]) / cycle_time

    cmd.joint_cmd[3 * leg_side].kp = 60
    cmd.joint_cmd[3 * leg_side + 1].kp = 60
    cmd.joint_cmd[3 * leg_side + 2].kp = 60
    cmd.joint_cmd[3 * leg_side].kd = 0.7
    cmd.joint_cmd[3 * leg_side + 1].kd = 0.7
    cmd.joint_cmd[3 * leg_side + 2].kd = 0.7
    cmd.joint_cmd[3 * leg_side].position = goal_angle[0]
    cmd.joint_cmd[3 * leg_side + 1].position = goal_angle[1]
    cmd.joint_cmd[3 * leg_side + 2].position = goal_angle[2]
    cmd.joint_cmd[3 * leg_side].velocity = goal_vel[0]
    cmd.joint_cmd[3 * leg_side + 1].velocity = goal_vel[1]
    cmd.joint_cmd[3 * leg_side + 2].velocity = goal_vel[2]
    for i in range(0, 12):
      cmd.joint_cmd[i].torque = 0
      

    
  def PreStandUp(self, cmd, time, data_state):
    standup_time = 2.0
    cycle_time = 0.001
    goal_angle_fl = [0 * kDegree2Radian, -70 * kDegree2Radian, 150 * kDegree2Radian]
    goal_angle_fr = [0 * kDegree2Radian, -70 * kDegree2Radian, 150 * kDegree2Radian]
    goal_angle_hl = [0 * kDegree2Radian, -70 * kDegree2Radian, 150 * kDegree2Radian]
    goal_angle_hr = [0 * kDegree2Radian, -70 * kDegree2Radian, 150 * kDegree2Radian]

    if time <= self.init_time + standup_time:
      self.SwingToAngle(self.init_angle_fl, goal_angle_fl, standup_time, time - self.init_time,
                        cycle_time, "FL", cmd, data_state)
      self.SwingToAngle(self.init_angle_fr, goal_angle_fr, standup_time, time - self.init_time,
                        cycle_time, "FR", cmd, data_state)
      self.SwingToAngle(self.init_angle_hl, goal_angle_hl, standup_time, time - self.init_time,
                        cycle_time, "HL", cmd, data_state)
      self.SwingToAngle(self.init_angle_hr, goal_angle_hr, standup_time, time - self.init_time,
                        cycle_time, "HR", cmd, data_state)
   
  def StandUp(self, cmd, time, data_state):
    standup_time = 2.5
    cycle_time = 0.001
    goal_angle_fl = [0 * kDegree2Radian, -42 * kDegree2Radian, 78 * kDegree2Radian] 
    goal_angle_fr = [0 * kDegree2Radian, -42 * kDegree2Radian, 78 * kDegree2Radian]
    goal_angle_hl = [0 * kDegree2Radian, -42 * kDegree2Radian, 78 * kDegree2Radian]
    goal_angle_hr = [0 * kDegree2Radian, -42 * kDegree2Radian, 78 * kDegree2Radian]

    if (time <= self.init_time + standup_time):
      self.SwingToAngle(self.init_angle_fl, goal_angle_fl, standup_time, time - self.init_time,
                        cycle_time, "FL", cmd, data_state)
      self.SwingToAngle(self.init_angle_fr, goal_angle_fr, standup_time, time - self.init_time,
                        cycle_time, "FR", cmd, data_state)
      self.SwingToAngle(self.init_angle_hl, goal_angle_hl, standup_time, time - self.init_time,
                        cycle_time, "HL", cmd, data_state)
      self.SwingToAngle(self.init_angle_hr, goal_angle_hr, standup_time, time - self.init_time,
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
    self.init_angle_fl[0] = data.joint_data[0].position
    self.init_angle_fl[1] = data.joint_data[1].position
    self.init_angle_fl[2] = data.joint_data[2].position

    self.init_angle_fr[0] = data.joint_data[3].position
    self.init_angle_fr[1] = data.joint_data[4].position
    self.init_angle_fr[2] = data.joint_data[5].position

    self.init_angle_hl[0] = data.joint_data[6].position
    self.init_angle_hl[1] = data.joint_data[7].position
    self.init_angle_hl[2] = data.joint_data[8].position

    self.init_angle_hr[0] = data.joint_data[9].position
    self.init_angle_hr[1] = data.joint_data[10].position
    self.init_angle_hr[2] = data.joint_data[11].position
