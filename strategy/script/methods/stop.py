#!/usr/bin/env python
from __future__ import print_function
import rospy
import math

class Stop(object):

  def ClassicStopping(self, goal_dis, goal_ang):
    v_x   = 0
    v_y   = 0
    v_yaw = 0

    return v_x, v_y, v_yaw
