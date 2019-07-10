#!/usr/bin/env python
import rospy
import math
import numpy as np
from nubot_common.msg import OminiVisionInfo
from nubot_common.srv import Shoot
from nubot_common.srv import BallHandle
from transfer.msg import PPoint
from simple_pid import PID
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from vision.msg import Object
from imu_3d.msg import inertia
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from std_msgs.msg import Int32

## Gazebo Simulator
SIM_VISION_TOPIC = "nubot{}/omnivision/OmniVisionInfo"
SIM_SHOOT_SRV  = "nubot{}/Shoot"
SIM_HANDLE_SRV = "nubot{}/BallHandle"

## Real Robot
VISION_TOPIC = "vision/object"
CMDVEL_TOPIC = "motion/cmd_vel"
SHOOT_TOPIC  = "motion/shoot"
POSITION_TOPIC = "akf_pose"

## Strategy Outputs
STRATEGY_STATE_TOPIC = "robot{}/strategy/state"

class Robot(object):

  __robot_info = {'location' : {'x' : 0, 'y' : 0, 'yaw' : 0}}
  __object_info = {'ball':{'dis' : 0, 'ang' : 0},
                   'Blue':{'dis' : 0, 'ang' : 0},
                   'Yellow':{'dis' : 0, 'ang' : 0},
                   'velocity' : 0 }
  ## Configs
  __minimum_w = 0.2
  __maximum_w = 100
  __minimum_v = 0
  __maximum_v = 50
  __handle_dis = 25
  __handle_ang = 5
  Kp_v = 1.5
  Ki_v = 0.0
  Kd_v = 0.1
  Cp_v = 20
  Kp_w = 0.25
  Ki_w = 0.0
  Kd_w = 0.0
  Cp_w = 0

  pid_v = PID(Kp_v, Ki_v, Kd_v, setpoint=Cp_v)
  pid_v.output_limits = (__minimum_v, __maximum_v)
  pid_v.auto_mode = True
  pid_w = PID(Kp_w, Ki_w, Kd_w, setpoint=Cp_w)
  pid_w.output_limits = (-1*__maximum_w, __maximum_w)
  pid_w.auto_mode = True

  def TuningVelocityContorller(self, p, i, d, cp = 20):
    self.pid_v.setpoint = cp
    self.pid_v.tunings = (p, i, d)

  def TuningAngularVelocityContorller(self, p, i, d, cp = 0):
    self.pid_w.setpoint = cp
    self.pid_w.tunings = (p, i, d)

  def ChangeVelocityRange(self, m, M):
    self.pid_v.output_limits = (-1*M, M)

  def ChangeAngularVelocityRange(self, m, M):
    self.pid_w.output_limits = (-1*M, M)

  def ChangeBallhandleCondition(self, dis, ang):
    self.__handle_dis = dis
    self.__handle_ang = ang

  def ShowRobotInfo(self):
    print("Robot informations: {}".format(self.__robot_info))
    print("Objects informations: {}".format(self.__object_info))

  def __init__(self, robot_num, sim = False):
    self.robot_number = robot_num

    if not sim :
      rospy.Subscriber(VISION_TOPIC, Object, self._GetVision)
      rospy.Subscriber(POSITION_TOPIC,PoseWithCovarianceStamped,self._GetPosition)
      self.MotionCtrl = self.RobotCtrlS
      self.RobotBallHandle = self.RealBallHandle
      self.RobotShoot = self.RealShoot
    else:
      self._SimSubscriber(SIM_VISION_TOPIC.format(self.robot_number))
      self.MotionCtrl = self.RobotCtrlS
      self.RobotBallHandle = self.SimBallHandle
      self.RobotShoot = self.SimShoot
      self.TuningVelocityContorller(1, 0, 0)
      self.TuningAngularVelocityContorller(0.1, 0, 0)

    self.cmdvel_pub = self._Publisher(CMDVEL_TOPIC, Twist)
    self.state_pub  = self._Publisher(STRATEGY_STATE_TOPIC.format(self.robot_number), String)
    self.shoot_pub  = self._Publisher(SHOOT_TOPIC, Int32)

  def _SimSubscriber(self, topic):
    rospy.Subscriber(topic.format(self.robot_number), \
                      OminiVisionInfo, \
                      self._GetSimVision)
    rospy.Subscriber((topic + "/GoalInfo").format(self.robot_number), \
                      PPoint, \
                      self._GetSimGoalInfo)

  def _Publisher(self, topic, mtype):
    return rospy.Publisher(topic, mtype, queue_size=1)

  def _GetSimVision(self, vision):
    self.__object_info['ball']['dis'] = vision.ballinfo.real_pos.radius
    self.__object_info['ball']['ang'] = math.degrees(vision.ballinfo.real_pos.angle)
    self.__robot_info['location']['x']   = vision.robotinfo[self.robot_number - 1].pos.x
    self.__robot_info['location']['y']   = vision.robotinfo[self.robot_number - 1].pos.y
    self.__robot_info['location']['yaw'] = math.degrees(vision.robotinfo[self.robot_number - 1].heading.theta)

  def _GetSimGoalInfo(self, goal_info):
    self.__object_info['Blue']['dis'] = goal_info.right_radius
    self.__object_info['Blue']['ang'] = goal_info.right_angle
    self.__object_info['Yellow']['dis']  = goal_info.left_radius
    self.__object_info['Yellow']['ang']  = goal_info.left_angle

  def _GetVision(self, vision):
    self.__object_info['ball']['dis']    = vision.ball_dis
    self.__object_info['ball']['ang']    = vision.ball_ang
    self.__object_info['Blue']['dis']    = vision.blue_fix_dis
    self.__object_info['Blue']['ang']    = vision.blue_fix_ang
    self.__object_info['Yellow']['dis']  = vision.yellow_fix_dis
    self.__object_info['Yellow']['ang']  = vision.yellow_fix_ang
  

  def _GetPosition(self,loc):
    self.__robot_info['location']['x'] = loc.pose.pose.position.x*100
    self.__robot_info['location']['y'] = loc.pose.pose.position.y*100
    qx = loc.pose.pose.orientation.x
    qy = loc.pose.pose.orientation.y
    qz = loc.pose.pose.orientation.z
    qw = loc.pose.pose.orientation.w
    self.__robot_info['location']['yaw'] = math.atan2(2 * (qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz) / math.pi * 180


  def RobotStatePub(self, state):
    s = String()
    s.data = state
    self.state_pub.publish(s)

  def Rotate(self, x, y, theta):
    _x = x*math.cos(math.radians(theta)) - y*math.sin(math.radians(theta))
    _y = x*math.sin(math.radians(theta)) + y*math.cos(math.radians(theta))
    return _x, _y

  def RobotCtrlS(self, x, y, yaw, pass_through=False):
    if pass_through:
      msg = Twist()
      msg.linear.x = -y
      msg.linear.y = x
      msg.angular.z  = yaw
      self.cmdvel_pub.publish(msg)
    else:
      current_vector = math.hypot(x, y)
      output_v = self.pid_v(current_vector * -1)
      output_w = self.pid_w(yaw) * -1
      output_w = output_w if abs(output_w) > self.__minimum_w else self.__minimum_w* np.sign(output_w)

      magnitude = math.sqrt(x**2 + y**2)
      if magnitude == 0:
        unit_vector = (0, 0)
      else:
        unit_vector = (x / magnitude, y / magnitude)

      msg = Twist()
      ## Rotate 90 for 6th robot
      output_x, output_y = self.Rotate(unit_vector[0]*output_v, unit_vector[1]*output_v, 90)

      msg.linear.x   = output_x
      msg.linear.y   = output_y
      msg.angular.z  = output_w
      self.cmdvel_pub.publish(msg)

  def GetObjectInfo(self):
    return self.__object_info

  def GetRobotInfo(self):
    return self.__robot_info

  def SimShoot(self, power, pos) :
    rospy.wait_for_service(SIM_SHOOT_SRV.format(self.robot_number))
    try:
      client = rospy.ServiceProxy(SIM_SHOOT_SRV.format(self.robot_number), Shoot)
      resp1 = client(power, pos)
      return resp1
    except rospy.ServiceException :
      print ("Service call failed")

  def RealShoot(self, power, pos) :
    msg = Int32()
    msg.data = 100
    self.shoot_pub.publish(msg)

  def SimBallHandle(self):
    rospy.wait_for_service(SIM_HANDLE_SRV.format(self.robot_number))
    try:
      client = rospy.ServiceProxy(SIM_HANDLE_SRV.format(self.robot_number), BallHandle)
      resp1 = client(1)
      return resp1.BallIsHolding
    except rospy.ServiceException :
      print ("Service call failed")

  def RealBallHandle(self):
    if self.__object_info['ball']['dis'] < self.__handle_dis and self.__object_info['ball']['ang'] < self.__handle_ang:
      return True
    else:
      return False
