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
     
    if a == 20 and abs( robot_info['location']['yaw'] ) < 20 :
      v_yaw = a - robot_info['location']['yaw']
      

    elif a == -20 and abs( robot_info['location']['yaw'] ) < 20 :
      v_yaw = -20 - robot_info['location']['yaw']
      

    elif a == 20 and abs( robot_info['location']['yaw'] ) >160 :
      v_yaw = 160 - robot_info['location']['yaw']
      

    elif a == -20 and abs( robot_info['location']['yaw'] )  >160 :
      v_yaw = robot_info['location']['yaw'] - 160
     


    else:
      pass


    return v_x, v_y, v_yaw

