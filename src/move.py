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
        self.viz = LocalMapVisualizer()

    def setGoal(self, goalXY):
        self.goal = goalXY
        self.paintGoal()

    def paintGoal(self):
        vOrigin = self.rp.origin()
        # figure out the vector to the goal
        vToGoal = vector_minus(self.goal, vOrigin)

        self.viz.vizPoints([vToGoal])

    # this is called by the main update loop of the program.  It uses the robot global compass
    # and laser interpreter to figure out where the obstacles are and move around
    def publishNextMovement(self):
        if not self.goal:
            # by default just move forward
            self.velPublish.publish(Twist(Vector3(.2,0,0),Vector3(0,0,0)))
            return

        # variables with a v prefix are vectors
        vOrigin = self.rp.origin()
        # figure out the robot position
        vForward = self.rp.forwardVector()

        # figure out the vector to the goal
        vToGoal = vector_minus(self.goal, vOrigin)

        #self.viz.vizPoints([vToGoal])
        self.viz.vizSegment([0,0,0], vToGoal, the_id=1)
        self.viz.vizSegment([0,0,0], vForward, the_id=2)
        
        rospy.loginfo("Theta: %0.2f Origin: [%0.2f, %0.2f] Goal: [%0.2f, %0.2f] vToGoal: [%0.2f, %0.2f]", r2d(self.rp.theta()), vOrigin[0], vOrigin[1],  self.goal[0], self.goal[1], vToGoal[0], vToGoal[1])
        



        # determine the angle between the the forward vector and the v
        signedAngleToGoal = vector_angle_signed(vForward, vToGoal)

        #rospy.loginfo("Theta: %0.2f", r2d(self.rp.theta()))
        #rospy.loginfo("Origin: [%0.2f, %0.2f] vToGoal: [%0.2f, %0.2f] Forward: [%0.2f, %0.2f]", vOrigin[0], vOrigin[1], vToGoal[0], vToGoal[1], vForward[0], vForward[1])

        MAX_ANGULAR_VELOCITY = d2r(10.0)
        MAX_LINEAR_VELOCITY = .10 # 50 cm

        angularVelocity = 0
        linearVelocity = Vector3(0,0,0)
        
        # if the angle is off by more than 5 degrees, just rotate
        if m


        ath.fabs(signedAngleToGoal) > d2r(80.0) and normalizeAngle90(signedAngleToGoal) > d2r(80.0):
            angularVelocity = signedAngleToGoal
            if math.fabs(signedAngleToGoal) > MAX_ANGULAR_VELOCITY:
               sign = (signedAngleToGoal >= 0 and 1.0) or -1.0
               angularVelocity = MAX_ANGULAR_VELOCITY * sign
               
            rospy.loginfo("Origin: [%0.2f, %0.2f] vToGoal: [%0.2f, %0.2f] Forward: [%0.2f, %0.2f]", vOrigin[0], vOrigin[1], vToGoal[0], vToGoal[1], vForward[0], vForward[1])
            rospy.loginfo("Theta: %0.2f degrees. Rotating the robot by %0.2f degrees. %0.2f degrees to goal", r2d(self.rp.theta()), r2d(angularVelocity), r2d(signedAngleToGoal))
            
        elif vector_length_squared(vToGoal) > .05:
            vVel = vToGoal
            if vector_length_squared(vVel) > MAX_LINEAR_VELOCITY:
                vVel = vector_scale(vector_normalize(vVel), MAX_LINEAR_VELOCITY)

            #rospy.loginfo("Setting velocity to [%0.2f, %0.2f]", vVel[0], vVel[1])
            linearVelocity = Vector3(vVel[0],vVel[1],0)
            
        # otherwise if the goal is kind of far away, set the forward velocity
        self.velPublish.publish(Twist(linearVelocity,Vector3(0,0,angularVelocity)))
