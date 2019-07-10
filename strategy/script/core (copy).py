#!/usr/bin/env python
import rospy
import sys
import math
import time
from statemachine import StateMachine, State
from robot.robot import Robot
from std_msgs.msg import String
from my_sys import log, SysCheck, logInOne
from methods.chase import Chase
from methods.stop import Stop
from methods.attack import Attack
from methods.penalty import Penalty
from methods.turn import Turn
from methods.behavior import Behavior
from dynamic_reconfigure.server import Server
from strategy.cfg import StrategyConfig
import dynamic_reconfigure.client
from time import sleep
import random

class Core(Robot, StateMachine):
  def __init__(self, robot_num, sim = False):
    super(Core, self).__init__(robot_num, sim)
    StateMachine.__init__(self)
    self.CC  = Chase()
    self.SC  = Stop()
    self.AC  = Attack()
    self.PC  = Penalty()
    self.TC  = Turn()
    self.BC  = Behavior()
    self.sim = sim
    

  idle     = State('Idle', initial = True)
  chase    = State('Chase')
  stop     = State('stop')
  penalty  = State('Penalty')
  turn     = State('turn')  
  attack   = State('Attack')
  shoot    = State('Shoot')
  orbit    = State('Orbit')
  point    = State('Point')

  toIdle    = chase.to(idle) | attack.to(idle)  | orbit.to(idle) | point.to(idle) | penalty.to(idle) | idle.to.itself() | shoot.to(idle) | stop.to(idle) | turn.to(idle)
  toChase   = idle.to(chase) | attack.to(chase) | chase.to.itself() | orbit.to(chase) | shoot.to(chase)
  toStop    = attack.to(stop) | penalty.to(stop) | turn.to(stop)
  toAttack  = chase.to(attack) | attack.to.itself() | orbit.to(attack)
  toPenalty = chase.to(penalty) | penalty.to.itself()
  toTurn    = chase.to(turn) | turn.to.itself()  
  toShoot   = stop.to(shoot) 
  toOrbit   = chase.to(orbit) | orbit.to.itself()
  toPoint   = point.to.itself() | idle.to(point)


  def on_toIdle(self):
    for i in range(0, 10):
        self.MotionCtrl(0,0,0)
    log("To Idle1")

  def on_toChase(self, t, side, method = "Classic"):
    if method == "Classic":
      x, y, yaw = self.CC.ClassicRounding(t[side]['ang'],\
                                          t['ball']['dis'],\
                                          t['ball']['ang'])
      self.MotionCtrl(x, y, yaw)
    elif method == "Straight":
      x, y, yaw = self.CC.StraightForward(t['ball']['dis'], t['ball']['ang'])
      self.MotionCtrl(x, y, yaw)
    else:
      pass


  def on_toAttack(self, t, side):
    x, y, yaw = self.AC.ClassicAttacking(t[side]['dis'], t[side]['ang'])
    self.MotionCtrl(x, y, yaw)

  def on_toStop(self, t, side):
    x, y, yaw = self.SC.ClassicStopping(t[side]['dis'], t[side]['ang'])
    self.MotionCtrl(x, y, yaw)


  def on_toPenalty(self, t, side):
    x, y, yaw = self.PC.ClassicPenalting(t[side]['dis'], t[side]['ang'])
    self.MotionCtrl(x, y, yaw)
    
  def on_toTurn(self, t , side ,p, location , a):
    x, y, yaw = self.TC.ClassicTurning(t[side]['dis'], t[side]['ang'], p['location']['yaw'], a)
    self.MotionCtrl(x, y, yaw)


  def on_toShoot(self, power, pos):
    self.RobotShoot(power, pos)


  def on_toOrbit(self, t, side):
    x, y, yaw = self.CC.Orbit(t[side]['ang'])
    self.MotionCtrl(x, y, yaw, True)

  def on_toPoint(self, tx, ty, tyaw):
    x, y, yaw, remaining = self.BC.Go2Point(tx, ty, tyaw)
    self.MotionCtrl(x, y, yaw)
    return remaining

  def PubCurrentState(self):
    self.RobotStatePub(self.current_state.identifier)

  def CheckBallHandle(self):
    return self.RobotBallHandle()

class Strategy(Robot):
  def __init__(self):
    self.game_start = False
    self.game_state = "Kick_Off"
    self.side       = "Yellow"
    self.opp_side   = 'Yellow' if self.side == 'Blue' else 'Blue'
    self.location   = 'location'
   
  def RunStatePoint(self, state):
    if state == "Kick_Off" :
      r = self.robot.toPoint(0, 0, 0)
    elif state == "Free_Kick" :
      r = self.robot.toPoint(100, 100, 90)
    elif state == "Free_Ball" :
      r = self.robot.toPoint(100, -100, 180)
    elif state == "Throw_In" :
      r = self.robot.toPoint(-100, -100, 270)
    elif state == "Coner_Kick":
      r = self.robot.toPoint(300, 200, 45)
    elif state == "Penalty_Kick" :
      r = self.robot.toPoint(-100, 100, 135)
    elif state == "Run_Specific_Point" :
      r = self.robot.toPoint(self.run_x, self.run_y, 0)
    else:
      print("ummmm")

    if r < 40:
      self.robot.toIdle()

  def Callback(self, config, level):
    self.game_start = config['game_start']
    self.game_state = config['game_state']
    self.run_point  = config['run_point']
    self.side       = config['our_goal']
    self.opp_side   = 'Yellow' if config['our_goal'] == 'Blue' else 'Blue'
    self.run_x      = config['run_x']
    self.run_y      = config['run_y']
    

    self.ChangeVelocityRange(config['minimum_v'], config['maximum_v'])
    self.ChangeAngularVelocityRange(config['minimum_w'], config['maximum_w'])
    self.ChangeBallhandleCondition(config['ballhandle_dis'], config['ballhandle_ang'])

    self.run_point = config['run_point']

    return config

  def main(self, argv):
    rospy.init_node('core', anonymous=True)
    rate = rospy.Rate(1000)
    b = input("b=")
    print(input)



    dsrv = Server(StrategyConfig, self.Callback)
    dclient = dynamic_reconfigure.client.Client("core", timeout=30, config_callback=None)

    TEST_MODE = True
    if SysCheck(argv) == "Native Mode":
      log("Start Native")
      self.robot = Core(1)
      
    elif SysCheck(argv) == "Simulative Mode":
      log("Start Sim")
      self.robot = Core(1, True)

    while not rospy.is_shutdown():
      a = 0

      self.robot.PubCurrentState()

      targets = self.robot.GetObjectInfo()
      position = self.robot.GetRobotInfo()

      if targets is None or targets['ball']['ang'] == 999 and self.game_start: # Can not find ball when starting
        print("Can not find ball")
        self.robot.toIdle()
      else:
        if not self.robot.is_idle and not self.run_point and not self.game_start:
          self.robot.toIdle()
          print("idle")
          print(position['location']['yaw'])

        if self.robot.is_idle and self.game_start:
                              
          dclient.update_configuration({"run_point": False})
          #self.robot.toChase(targets, self.opp_side, "Straight")
          self.robot.toChase(targets, self.opp_side, "Straight")
          
          
          print("chase")
          # go chase

        elif self.robot.is_chase and not self.robot.CheckBallHandle() :
                       
          #self.robot.toChase(targets, self.opp_side, "Straight")
          self.robot.toChase(targets, self.opp_side, "Straight")
          
          print("keep chase")
          # keep chase


        if self.robot.is_chase and self.robot.CheckBallHandle() \
                               and b == 1:
	  
          self.robot.toPenalty(targets, self.opp_side)
          print("penalty")
          # go penalty
        
        if self.robot.is_chase and self.robot.CheckBallHandle() \
                               and b == 2:
          
          a = random.choice([20,-20])
	  
          self.robot.toTurn(targets, self.opp_side , position, self.location , a)
          print("turn")
          # go turn 

        if self.robot.is_chase and self.robot.CheckBallHandle() \
                               and b == 0:
	  
          self.robot.toAttack(targets, self.opp_side)
          print("attack")
          # go attack

        if self.robot.is_penalty and self.robot.CheckBallHandle() \
                                 and abs(targets[self.opp_side]['ang'])  > 5 \
                                 and b == 1:

          self.robot.toPenalty(targets, self.opp_side)
          print("keep penalty")
          # keep  penalty

        if self.robot.is_turn and self.robot.CheckBallHandle() \
                              and abs(position[self.location]['yaw']) < 20 or abs(position[self.location]['yaw']) > 160 \
                              and a == 20\
                              and b == 2:
          a = 20

          self.robot.toTurn(targets, self.opp_side , position, self.location , a)
          print("keep turn")
          # keep  turn

        elif self.robot.is_turn and self.robot.CheckBallHandle() \
                                and abs(position[self.location]['yaw']) < 20 or abs(position[self.location]['yaw']) > 160 \
                                and a == -20\
                                and b == 2:
          a = -20

          self.robot.toTurn(targets, self.opp_side , position, self.location , a)
          print("keep turn")
          # keep  turn


        if self.robot.is_penalty and self.robot.CheckBallHandle() \
                                 and abs(targets[self.opp_side]['ang'])  <=5 :

          self.robot.toStop(targets, self.opp_side)
          print("stop")
          # stop

        if self.robot.is_turn and self.robot.CheckBallHandle() \
                              and abs(position[self.location]['yaw']) >= 20 :

          self.robot.toStop(targets, self.opp_side)
          print("stop")
          sleep(1)
          print(position['location']['yaw'])
          # stop 

        if self.robot.is_turn and self.robot.CheckBallHandle() \
                              and abs(position[self.location]['yaw']) == 160 :

          self.robot.toStop(targets, self.opp_side)
          print("stop")
          sleep(1)
          print(position['location']['yaw'])
          # stop 
        

        elif self.robot.is_attack and self.robot.CheckBallHandle() \
                                  and targets[self.opp_side]['dis'] >= 150:
          self.robot.toAttack(targets, self.opp_side)
          print("keep attack")
          # keep attack

        elif self.robot.is_attack and not self.robot.CheckBallHandle() :
                                  
          self.robot.toChase(targets, self.opp_side, "Straight")
          print("back to chase")
          # back to chase

        elif self.robot.is_attack and self.robot.CheckBallHandle() \
                                  and abs(targets[self.opp_side]['ang']) < 10 \
                                  and targets[self.opp_side]['dis'] <= 150:
          self.robot.toStop(targets, self.opp_side)
          print("stop")
          # stop

        if self.robot.is_stop and self.robot.CheckBallHandle():
          self.robot.toShoot(100, 1)
          b = 0
          print("shoot")
          # shoot

        if self.robot.is_shoot and self.robot.CheckBallHandle():
          sleep(1)
          # stop for three seconds
          self.robot.toChase(targets, self.opp_side, "Straight")
          print("back to chase")
          # back to chase

      ## Run point
      if self.run_point and not self.game_start:
        self.RunStatePoint(self.game_state)
        if self.robot.is_point:
          self.RunStatePoint(self.game_state)

      if rospy.is_shutdown():
        log('shutdown')
        break

      rate.sleep()

if __name__ == '__main__':
  try:
    s = Strategy()
    s.main(sys.argv[1:])
  except rospy.ROSInterruptException:
    pass
