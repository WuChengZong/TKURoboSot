#!/usr/bin/env python
from __future__ import print_function
import rospy
import math
from robot.robot import Robot

class Penalty(object):

  def ClassicPenalting(self, goal_dis, goal_ang):
    v_x   = 0
    v_y   = 0
    v_yaw = goal_ang


    #while abs(goal_ang) == 45:
     ## b = a - goal_ang
      #v_yaw = b

    return v_x, v_y, v_yaw

