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
        self.occGrid = OccupancyGrid(self.li)

    def setGoal(self, goalXY):
        self.goal = goalXY
        self.paintGoal()

    def paintGoal(self):
        vOrigin = self.rp.origin()
        # figure out the vector to the goal
        vToGoal = vector_minus(self.goal, vOrigin)

        self.viz.vizPoints([vToGoal])

    # returns the vector from the robot to the goal
    def vectorToGoal(self):
        vOrigin = self.rp.origin()
        vToGoal = vector_minus(self.goal, vOrigin)
        return vToGoal

    def robotRadius(self):
        return .2 # 1.5 feet sounds not too bad for the width

    def maxLinearVelocity(self):
        return .5 # half a meter per second

    def publishNextVelocityAvoidingObstaclesOccGrid(self):
        self.occGrid.updateGrid()
    def publishNextVelocityAvoidingObstacles(self):
        # figure out which way the goal is
        vToGoal = self.vectorToGoal()
        # now figure out how fast we want to go
        vToGoal = vector_scale(vector_normalize(vToGoal), self.maxLinearVelocity())
        goalAngle = vector_angle_general([0,-1], vToGoal)
        speed = vector_length(vToGoal)
        # we should be calculating distance based on the maximum stopping acceleration of the
        # robot, but we are winging it in favor of a simple solution that may work a little
        distance = speed * 2.0
        # now we figure out if we can actually go in this direction or if we will collide with something.
        # we do this by scanning
        # for now we just look 10 degrees to either side
        def ensureTrajectoryDoesNotCollide(vCandidateVelocity, log=False):
            angleWithAxis = vector_angle_general([0,-1], vCandidateVelocity)
            # scan 20 degrees for now
            minDistance = 10000.0
            for angleNum in range(-10, 11):
                angleToCheck = angleWithAxis + float(angleNum) * d2r(1.5)
                distanceToProbePoint = self.li.castRayPolar(angleToCheck)
                if distanceToProbePoint < minDistance:
                    minDistance = distanceToProbePoint

                if log:
                    rospy.loginfo("ACCEPTED subangle [%0.2f].probe angle [%0.2f] with range [%0.2f]", r2d(angleWithAxis), r2d(angleToCheck), distanceToProbePoint)
                if distanceToProbePoint < distance + self.robotRadius():
                    return False
            rospy.loginfo("ACCEPTED angle [%0.2f] min distance  [%0.2f]. velocity: [%0.2f, %0.2f]", r2d(angleWithAxis), minDistance, vCandidateVelocity[0], vCandidateVelocity[1])
            if not log:
                ensureTrajectoryDoesNotCollide(vCandidateVelocity, log=True)
            return True

        trajectory = None
        # try angles away from the goal N degrees at a time
        for angleNum in range(0, 22):
            angleToCheck = None
            if angleNum == 0:
                angleToCheck = goalAngle
            elif angleNum < 12:
                i = angleNum -1
                angleToCheck = goalAngle + float(i) * d2r(-8.0)
            else: #if angleNum % 2 == 0:
                i = angleNum - 12
                angleToCheck = goalAngle + float(i) * d2r(8.0)
            candidate = polarToCartesian(speed, angleToCheck)
            if ensureTrajectoryDoesNotCollide(candidate):
                trajectory = candidate
                break
        if trajectory:
            self.easyPublishVelocity(trajectory)

    # give it a velocity vector and let it figure out how to turn and all
    def easyPublishVelocity(self, velocity):
        # variables with a v prefix are vectors
        vOrigin = self.rp.origin()
        # figure out the robot position
        vForward = self.rp.forwardVector()
        vToGoal = self.vectorToGoal()
        # figure out the vector to the goal

        #self.viz.vizSegment([0,0,0], vToGoal, the_id=1)
        self.viz.vizSegment([0,0,0], vector_rotate_2d( vToGoal, -1.0 * self.rp.theta()), the_id=1)
        self.viz.vizSegment([0,0,0], vector_rotate_2d( velocity, -1.0 * self.rp.theta()), the_id=2)
        
        # determine the angle between the the forward vector and the v
        signedAngleToGoal = vector_angle_signed(vForward, velocity)

        #DEBUG
        rospy.loginfo("Theta: %0.2f Origin: [%0.2f, %0.2f] Goal: [%0.2f, %0.2f] vToGoal: [%0.2f, %0.2f]. AngTogoal: %0.2f degrees", r2d(self.rp.theta()), vOrigin[0], vOrigin[1],  self.goal[0], self.goal[1], vToGoal[0], vToGoal[1], r2d(signedAngleToGoal))
        #rospy.loginfo("Theta: %0.2f", r2d(self.rp.theta()))
        #rospy.loginfo("Origin: [%0.2f, %0.2f] vToGoal: [%0.2f, %0.2f] Forward: [%0.2f, %0.2f]", vOrigin[0], vOrigin[1], vToGoal[0], vToGoal[1], vForward[0], vForward[1])

        # restrict the angular velocity and the linear velocity
        MAX_ANGULAR_VELOCITY = d2r(10.0)
        MAX_LINEAR_VELOCITY = self.maxLinearVelocity() # 50 cm

        angularVelocity = 0
        linearVelocity = Vector3(0,0,0)
        
        # if the angle is off by more than 5 degrees, just rotate
        if math.fabs(signedAngleToGoal) > d2r(5.0) and normalizeAngle90(signedAngleToGoal) > d2r(5.0):
            angularVelocity = signedAngleToGoal
            if math.fabs(signedAngleToGoal) > MAX_ANGULAR_VELOCITY:
               sign = (signedAngleToGoal >= 0 and 1.0) or -1.0
               angularVelocity = MAX_ANGULAR_VELOCITY * sign
            #rospy.loginfo("Origin: [%0.2f, %0.2f] vToGoal: [%0.2f, %0.2f] Forward: [%0.2f, %0.2f]", vOrigin[0], vOrigin[1], vToGoal[0], vToGoal[1], vForward[0], vForward[1])
            rospy.loginfo("Theta: %0.2f degrees. Rotating the robot by [%0.2f] of [%0.2f] degrees to goal", r2d(self.rp.theta()), r2d(angularVelocity), r2d(signedAngleToGoal))
            
        if vector_length_squared(vToGoal) > .05:
            vVel = velocity
            if vector_length_squared(vVel) > MAX_LINEAR_VELOCITY:
                vVel = vector_scale(vector_normalize(vVel), MAX_LINEAR_VELOCITY)

            #rospy.loginfo("Setting velocity to [%0.2f, %0.2f]", vVel[0], vVel[1])
            linearVelocity = Vector3(vVel[0],vVel[1],0)
            
        # otherwise if the goal is kind of far away, set the forward velocity
        self.velPublish.publish(Twist(linearVelocity,Vector3(0,0,angularVelocity)))
        
        
    # this is called by the main update loop of the program.  It uses the robot global compass
    # and laser interpreter to figure out where the obstacles are and move around
    def publishNextMovement(self):
        if not self.goal:
            # by default just move forward
            self.velPublish.publish(Twist(Vector3(.2,0,0),Vector3(0,0,0)))
            return
        else:
            #self.publishNextVelocityAvoidingObstacles()
            self.publishNextVelocityAvoidingObstaclesOccGrid()
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
        
        # determine the angle between the the forward vector and the v
        signedAngleToGoal = vector_angle_signed(vForward, vToGoal)

        #DEBUG
        rospy.loginfo("Theta: %0.2f Origin: [%0.2f, %0.2f] Goal: [%0.2f, %0.2f] vToGoal: [%0.2f, %0.2f]. AngTogoal: %0.2f degrees", r2d(self.rp.theta()), vOrigin[0], vOrigin[1],  self.goal[0], self.goal[1], vToGoal[0], vToGoal[1], r2d(signedAngleToGoal))
        #rospy.loginfo("Theta: %0.2f", r2d(self.rp.theta()))
        #rospy.loginfo("Origin: [%0.2f, %0.2f] vToGoal: [%0.2f, %0.2f] Forward: [%0.2f, %0.2f]", vOrigin[0], vOrigin[1], vToGoal[0], vToGoal[1], vForward[0], vForward[1])

        # restrict the angular velocity and the linear velocity
        MAX_ANGULAR_VELOCITY = d2r(10.0)
        MAX_LINEAR_VELOCITY = self.maxLinearVelocity() # 50 cm

        angularVelocity = 0
        linearVelocity = Vector3(0,0,0)
        
        # if the angle is off by more than 5 degrees, just rotate
        if math.fabs(signedAngleToGoal) > d2r(5.0) and normalizeAngle90(signedAngleToGoal) > d2r(5.0):
            angularVelocity = signedAngleToGoal
            if math.fabs(signedAngleToGoal) > MAX_ANGULAR_VELOCITY:
               sign = (signedAngleToGoal >= 0 and 1.0) or -1.0
               angularVelocity = MAX_ANGULAR_VELOCITY * sign
            #rospy.loginfo("Origin: [%0.2f, %0.2f] vToGoal: [%0.2f, %0.2f] Forward: [%0.2f, %0.2f]", vOrigin[0], vOrigin[1], vToGoal[0], vToGoal[1], vForward[0], vForward[1])
            rospy.loginfo("Theta: %0.2f degrees. Rotating the robot by [%0.2f] of [%0.2f] degrees to goal", r2d(self.rp.theta()), r2d(angularVelocity), r2d(signedAngleToGoal))
            
        if vector_length_squared(vToGoal) > .05:
            vVel = vToGoal
            if vector_length_squared(vVel) > MAX_LINEAR_VELOCITY:
                vVel = vector_scale(vector_normalize(vVel), MAX_LINEAR_VELOCITY)

            #rospy.loginfo("Setting velocity to [%0.2f, %0.2f]", vVel[0], vVel[1])
            linearVelocity = Vector3(vVel[0],vVel[1],0)
            
        # otherwise if the goal is kind of far away, set the forward velocity
        self.velPublish.publish(Twist(linearVelocity,Vector3(0,0,angularVelocity)))

def mapFloatIntoDiscretizedBucket(f, minFloat, maxFloat, numBuckets):
    # f prefix float i discrete
    fSizeOfBucket = float(maxFloat - minFloat) / float(numBuckets)
    iBucket = int( float(f - minFloat) / fSizeOfBucket)
    if iBucket < 0:
        return 0
    elif iBucket >= numBuckets:
        return numBuckets - 1
    else:
        return iBucket

# findBestNewVelocities:
#   Takes in current x, y, theta, velocity v, angular velocity w, timestep dt, grid g
#   Calculates a cost estimate of each new v and w value within a reasonable range
def findBestNewVelocities(x, y, theta, v, w, dt, g, goal):
      maxAccel = 0.5 * dt  # 0.5 was pretty much randomly chosen
      maxAngularAccel = 0.5 * dt
      numIntervals = 20  # 20 x 20 sample grid = 400 test points
      minCost = 10000
      threshold = None  # no threshold to start
      bestVals = [0,0]
      for newV in arange(v-maxAccel, v+maxAccel, 2*maxAccel/numIntervals):
          for newW in arange(w-maxAngularAccel, w+maxAngularAccel, 2*maxAngularAccel/numIntervals):
              newTheta = theta + newW*dt  # calculate theta after dt
              halfwayTheta = (newTheta + theta)/2.0  # find the halfway theta
              newX = x + newV*cos(halfwayTheta)*dt  # estimate the x position after dt
              newY = y + newV*sin(halfwayTheta)*dt  # estimate the y position after dt
              cost = calculateCost(newX, newY, newV, g, goal)  # calculate the cost of this position
              if cost < minCost:  # store the best cost yet found
                  minCost = cost
                  bestVals = [newV, newW]
                  if threshold and cost < threshold:  # optionally, provide a threshold to stop at
                      return bestVals
      return bestVals

# calculateCost:
#   Estimates the cost of a given (x,y,v) state on an occupancy grid g
def calculateCost(x, y, v, g, goal):
    obsCoefficient = 2.0  # Assign some multipliers for the costs
    goalCoefficient = 3.0
    speedCoefficient = 1.0
    obsCost = g.distToNearestObstacle(x, y)
    goalCost = (goal[0] - x) + (goal[1] - y)
    speedCost = 1.0 / v
    return obsCoefficient*obsCost + goalCoefficient*goalCost + speedCoefficient*speedCost

class OccupancyGrid:
    def __init__(self, li):
        self.li = li # laser interpreter
        self.minFloat = -2
        self.maxFloat = 2
        self.bucketsPerDimension = 10
        self.grid = map (lambda x : 1, range(0, self.bucketsPerDimension * self.bucketsPerDimension))

    #givena  tuple containing floats, returns the value in the occupancy grid 
    def getGridValue(self, point):
        [x, y] = self.pointToBucketXY(point)
        return self.getBucketValue( x, y)
    
    def setGridValue(self, point, value):
        [x, y] = self.pointToBucketXY(point)
        self.setBucketValue(x, y, value)
                                                                                                            # returns integer [x, y] of the bucket that corresponds to the given floating point values
    def setBucketValue(self, x, y, value):
        self.grid[x * self.bucketsPerDimension + y] = value
        
    def getBucketValue(self, x, y):
        return self.grid[x * self.bucketsPerDimension + y]

    def pointToBucketXY(self, pt):
        [x, y] = pt
        # if we are looking for -2 1 in a grid, we would return 
        xBucket = mapFloatIntoDiscretizedBucket(x, self.minFloat, self.maxFloat, self.bucketsPerDimension)
        yBucket = mapFloatIntoDiscretizedBucket(y, self.minFloat, self.maxFloat, self.bucketsPerDimension)
        return [xBucket, yBucket]

    def calculateSpacing(self):
        return float(self.maxFloat - self.minFloat) / float(self.bucketsPerDimension)

    # update the grid with the laser scan info
    def updateGrid(self):
        step = self.calculateSpacing()
        for iX in range(0, self.bucketsPerDimension):
            fX = float(iX) + step * .5 + self.minFloat
            for iY in range(0, self.bucketsPerDimension):
                fY = float(iY) + step * .5 + self.minFloat
                if fY > 0 and True:
                    vec = [fX, fY]
                    cast_distance = self.li.castVector(vec)
                    # 0 weight for 
                    gridVal = 0.0
                    if cast_distance * cast_distance > vector_length_squared(vec):
                        gridVal = 1.0
                    self.setGridValue(vec, gridVal)

    # distToNearestObstacle:
    #   An occupancy grid function - probably want this in the class
    #   Does an inefficient search for the nearest obstance in the grid
    def distToNearestObstacle(self, startX, startY):
        queue = [ [startX, startY] ]
        checked = { [startX, startY] : True }
        spacing = [self.calculateSpacing(), self.calculateSpacing()]  # SYNTAX CHECK: needs the cell spacing
        while len(queue) > 0:
            [x, y] = queue.pop(0)
            
            if self.getGridValue([x, y]) < .1:  # SYNTAX CHECK: needs to check for obstacle in cell (x,y)
                return x + y  # found an obstacle, return Manhattan distance

            for i in [-spacing[0], spacing[0]]:  # spread out from the current cell
                for j in [-spacing[1], spacing[1]]:
                    if not [x+i, y+j] in checked:  # check if this cell is in the dictionary
                        queue.append[x+i, y+j]
                        checked[ [x+i, y+j] ]= True
