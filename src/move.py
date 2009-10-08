import roslib;
import rospy
import tf
from math import *
from line import *
from util import *
from vector import *
from lineviz import *
from robotPosition import *


class MoveToGoal:
    def __init__(self, robotPosition, laserInterpreter):
        self.xGoal = 4  # 4m
        self.thetaGoal = 180  # 180 degrees
        self.rp = robotPosition  # the RobotPosition
        self.li = laserInterpreter
        self.velPublish = rospy.Publisher("commands", Twist) # publish to "commands"
        self.stage = 1 # go through 4 stages: 1. go fwd 10 feet. 2. turn around. 3.

    def setGoal(self, goalXY):
        self.goal = goalXY
    # this is called by the main update loop of the program.  It uses the robot global compass
    # and laser interpreter to figure out where the obstacles are and move around
    def publishNextMovement(self):
        if not self.goal:
            return
        fwd = self.rp.forwardVector()
        
        linearVel = Vector3(0,0,0)
        angularVel = Vector3(0,0,4.0/r2d)
        
        self.velPublish.publish(Twist(Vector3(0,0,0),Vector3(0,0,4.0/r2d)))
        xVel = 0
        thetaVel = 0
        if self.stage == 1:  # haven't gone forward enough
            xVel = 0.5
        if rp.trans[0] >= self.xGoal:
            self.stage += 1
        elif self.stage == 2:  # haven't turned enough
            thetaVel = 10.0/r2d  # turn 10 degrees (per second?)
            if rp.rot >= self.thetaGoal - 1:
                self.stage += 1
        elif self.stage == 3:
            thetaVel = 1.0/r2d
            if rp.rot >= self.thetaGoal - 0.01:
                self.stage += 1
        elif self.stage == 4:
            xVel = 0.5
            if rp.trans[0] <= 0:
                self.stage += 1
        twist = Twist(Vector3(xVel, 0, 0), Vector3(0, 0, thetaVel))
        self.velPublish.publish(twist)

