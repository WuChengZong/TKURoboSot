#!/usr/bin/env python
from __future__ import print_function
import rospy
import math
from robot.robot import Robot

class Turn(Robot):

  def __init__(self):
    pass


  def ClassicTurning(self, x, y, yaw , a):
    robot_info = self.GetRobotInfo()
    v_x   = 0
    v_y   = 0
    v_yaw = 0
    print(a)
     
    if a >= 0 and abs( robot_info['location']['yaw'] ) < 90 :
      location = robot_info['location']['yaw']
      v_yaw = a - location      

    elif a <= 0 and abs( robot_info['location']['yaw'] ) < 90 :
      location = robot_info['location']['yaw']
      v_yaw = a - location
      

    elif a >= 0 and abs( robot_info['location']['yaw'] ) > 90 :
      location = robot_info['location']['yaw'] + 180
      v_yaw = a - location
      

    elif a <= 0 and abs( robot_info['location']['yaw'] )  >90 :
      location = robot_info['location']['yaw'] - 180
      v_yaw = a -location


    else:
      pass


    return v_x, v_y, v_yaw

