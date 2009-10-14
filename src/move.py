import roslib;
import rospy
import tf
from math import *
from line import *
from util import *
from vector import *
from lineviz import *
from robotPosition import *

viz = None


class Trajectory:
    MAX_VEL = 0.9
    MAX_ANG_VEL = math.pi
    MAX_ACCEL = 0.5
    MAX_ANG_ACCEL = math.pi / 1.6
    SIMULATION_DT = .3
    # constructs a velocity from provided robot controls (forward velocity and angular velocity)
    #
    # forward velocity is a scalar value in meters per second and is the velocity of the robot in
    #   the positive X direction
    #
    # angularVelocity is a scalar value in radians per second and it corresponds to the rotation
    #   of the robot about its center of rotation
    def __init__(self, forwardVelocity, angularVelocity, currentFwdVel, currentAngVel):
        self.fwdVel = forwardVelocity
        self.angVel = angularVelocity
        self.currentFwdVel = currentFwdVel
        self.currentAngVel = currentAngVel

    def vizTrajectory(self, name=None, color=None):
        positions = self.simulate(Trajectory.SIMULATION_DT, 2.0)
        positions = [[0.0,0.0]] + positions
        #displayVectors = map(vector_minus, positions[1:], positions[: len(positions)-1])
        positions = map(lambda xy: (xy[0], xy[1], .2), positions)
        rospy.loginfo("Visualizing Trajectory (%0.2f, %0.2f): %s", self.fwdVel, self.angVel, positions)
        viz.vizSegments(positions, name=name)

    def cost(self, simulationTime, occGrid, goal):
        positions = self.simulate(Trajectory.SIMULATION_DT, simulationTime)
        def costAtPosition(xy):
            return occGrid.obstacleCostAt(xy[0], xy[1])

        # average trajectory object collision costs
        allObsCosts = map(costAtPosition, positions)
        ocost = sum(allObsCosts) / float(len(allObsCosts))

        # calculate distanceToGoal from endpoint of trajectory
        lastPosition = positions[len(positions) - 1]
        gcost = occGrid.distToGoal(lastPosition[0], lastPosition[1])

        # speed cost is inversely proportional to the speed we are going
        scost = -1.0 * vector_dot([self.fwdVel, 0], vector_normalize(goal))

        ocoef = 3.0
        gcoef = 3.0
        scoef = 1.0
        totalCost = scost * scoef + ocost * ocoef + gcost * gcoef

        rospy.loginfo("all obstacle costs: %s", map(lambda x : "%0.2f" % x, allObsCosts))
        rospy.loginfo("all positions     : %s", map(lambda x : "(%0.2f, %0.2f)" % (x[0], x[1]), positions))
        rospy.loginfo("all cost at 0, 0: %s, bucket: %s", occGrid.obstacleCostAt(0.0, 0.0), occGrid.pointToBucketXY([0,0]))
        #
        rospy.loginfo("Trajectory (%0.2f, %0.2f) Cost: %s / speed: %0.2f / goal: %0.2f / obstac: %0.2f",
                      self.fwdVel, self.angVel, totalCost, scost * scoef, gcost * gcoef, ocost * ocoef)

        return totalCost
            
    # given a time period T, calculates where the robot will be in [x,
    # y] coordinates relative to the initial position of the robot
    # with time steps DT.  Assumes that the angular velocity will be
    # constantly applied over this period of time given a returns
    def simulate(self, dt, simulationTime):
        # invariants
        maxDeltaFwdVel = Trajectory.MAX_ACCEL * dt
        maxDeltaAngVel = Trajectory.MAX_ANG_ACCEL * dt
        # change with each step
        theta = 0.0
        [x, y] = [0.0, 0.0]
        fwdVel = self.currentFwdVel
        angVel = self.currentAngVel
        t = dt
        positions = []
        while t < simulationTime + .0001:
            # set the change in angular and forward velocity and clamp it to max
            deltaVel = self.fwdVel - fwdVel
            deltaAngVel = self.angVel - angVel

            if deltaVel < -maxDeltaFwdVel:
                deltaVel = -maxDeltaFwdVel
            elif deltaVel > maxDeltaFwdVel:
                deltaVel = maxDeltaFwdVel
            
            if deltaAngVel < -maxDeltaAngVel:
                deltaAngVel = -maxDeltaAngVel
            elif deltaAngVel > maxDeltaAngVel:
                deltaAngVel = maxDeltaAngVel

            angVel += deltaAngVel
            fwdVel += deltaVel
            
            newTheta = theta + dt * angVel  # calculate theta after dt
            halfwayTheta = (newTheta + theta)/2.0  # find the halfway theta
            theta = newTheta
            
            x = x + fwdVel*cos(halfwayTheta)*dt  # estimate the x position after dt
            y = y + fwdVel*sin(halfwayTheta)*dt  # estimate the y position after dt
            t += dt
            positions = positions + [ [x, y] ]
              
        return positions

class MoveToGoal:
    def __init__(self, robotPosition, laserInterpreter):
        global viz
        self.xGoal = 4  # 4m
        self.thetaGoal = 180  # 180 degrees
        self.rp = robotPosition  # the RobotPosition
        self.li = laserInterpreter
        self.velPublish = rospy.Publisher("commands", Twist) # publish to "commands"
        self.stage = 1 # go through 4 stages: 1. go fwd 10 feet. 2. turn around. 3.
        self.viz = LocalMapVisualizer()
        self.occGrid = OccupancyGrid(self.li, self.viz)
        viz = self.viz
    def setGoal(self, goalXY):
        self.goal = goalXY
        self._goalSafe = goalXY
        self.paintGoal()

    def paintGoal(self):
        vOrigin = self.rp.origin()
        # figure out the vector to the goal
        vToGoal = self.vectorToGoal()

        self.viz.vizPoints([vToGoal])

    # returns the vector from the robot to the goal
    def vectorToGoal(self):
        # vOrigin and vToGoal are in global coordinates
        return self.rp.globalToLocal(self._goalSafe)

    def robotRadius(self):
        return .2 # 1.5 feet sounds not too bad for the width

    def maxLinearVelocity(self):
        return .5 # half a meter per second

    def publishNextVelocityAvoidingObstaclesOccGrid(self):
        
        #rotated_vForward = vector_rotate_2d( vForward, -1.0 * self.rp.theta())
        rospy.loginfo("theta in degrees: %0.2f", r2d(self.rp.theta()))
        self.viz.vizSegment([0.0,0.0,0.0], [1.0,0.0,0.0], name="forward", color=[1.0, 1.0, 1.0])
        self.viz.vizSegment([0.0,0.0,0.0], self.vectorToGoal(), name="vToGoal", color=[0.0,0.0,1.0])
        #self.viz.vizSegment([0,0,0], rotated_vForward, name="vForward")

        # update the occupancy grid
        self.occGrid.updateGrid(self.vectorToGoal())

        # find the best trajectory
        bestTraj = self.findBestTrajectory()
        bestTraj.vizTrajectory("BestTrajectory", color=(1.0, 0.0, 0.0))

        newWVel = bestTraj.angVel
        newXVel = bestTraj.fwdVel

        self.velPublish.publish( Twist(Vector3(newXVel, 0, 0),Vector3(0, 0, newWVel)) )

    def findBestTrajectory(self):
        bestTraj = None
        bestCost = 1000000
        for traj in self.trajectoryGenerator():
            c = traj.cost(1.5, self.occGrid, self.goal)
            rospy.loginfo("Trajectory (%0.2f, %0.2f) has cost %0.2f", traj.fwdVel, traj.angVel, c)
            if c < bestCost or not bestTraj:
                bestCost = c
                bestTraj = traj
        rospy.loginfo("Best Trajectory (%0.2f, %0.2f) with cost %0.2f", bestTraj.fwdVel, bestTraj.angVel, bestCost)
        return bestTraj
        
    def trajectoryGenerator(self):
        minVel = 0.0
        maxVel = 0.5
        minAngVel = -pi / 4.0
        maxAngVel =  pi / 4.0
        velSteps = 3
        angSteps = 10
        trajNum = 0
        # derived values
        velStep = (maxVel - minVel) / float(velSteps)
        angVelStep = (maxAngVel - minAngVel) / float(angSteps)
        for i in xrange(0, velSteps):
            fwdVel = float(i) * velStep + minVel
            for j in xrange(0, angSteps):
                trajNum += 1
                angVel = float(j) * angVelStep + minAngVel
                # TODO actually get current angular and fwd velocities
                traj = Trajectory(fwdVel, angVel, .2, 0)
                traj.vizTrajectory(name="traj %i" % i)
                yield traj
        
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
      maxAccel = 0.2 * dt  # 0.5 was pretty much randomly chosen
      maxAngularAccel = math.pi / 5.0 * dt
      numIntervals = 10  # 20 x 20 sample grid = 400 test points
      minCost = 10000
      threshold = None  # no threshold to start
      bestVals = [0,0]
      newV = v - maxAccel  # initialize forward velocity
      attemptCount = 0
      while newV <= v+maxAccel:  # loop over all forward solutions
          newW = w - maxAngularAccel  # initialize angular velocity
          while newW <= w+maxAngularAccel:  # loop over all angular solutions
              newTheta = theta + newW*dt  # calculate theta after dt
              halfwayTheta = (newTheta + theta)/2.0  # find the halfway theta
              newX = x + newV*cos(halfwayTheta)*dt  # estimate the x position after dt
              newY = y + newV*sin(halfwayTheta)*dt  # estimate the y position after dt
              #rospy.loginfo("xVel = %0.2f     thetaVel = %0.2f", newV, newW)
              attemptCount = attemptCount + 1
              costs = calculateCost(x, y, newX, newY, newV, g, goal, name=attemptCount)  # calculate the cost of this position
              if costs[0] < minCost:  # store the best cost yet found
                  minCost = costs[0]
                  bestVals = [newV, newW]
                  rospy.loginfo("BETTER (%0.2f, %0.2f) obj: %0.2f  goal: %0.2f  speed: %0.2f  total: %0.2f  xvel: %0.2f  thetavel: %0.2f", newX, newY, costs[1], costs[2], costs[3], costs[0], bestVals[0], bestVals[1])
                  if threshold and cost < threshold:  # optionally, provide a threshold to stop at
                      return bestVals
              newW += 2*maxAngularAccel/numIntervals  # increment angular velocity
          newV += 2*maxAccel/numIntervals  # increment forward velocity
      rospy.loginfo("BEST obj: %0.2f  goal: %0.2f  speed: %0.2f  total: %0.2f  xvel: %0.2f  thetavel: %0.2f", costs[1], costs[2], costs[3], costs[0], bestVals[0], bestVals[1])
      return bestVals

# calculateCost:
#   Estimates the cost of a given (x,y,v) state on an occupancy grid g
#   the lower the cost, the more desirable
def calculateCost(oldX, oldY, x, y, v, g, goal, name=None):
    # segment the displacement vector into lengths .05 long and
    def calcObjsCost(vec):
        ocost = g.distToNearestObstacle(vec[0], vec[1])
        if ocost < 0.2:  # too close to an obstacle -- invalidate this option
            ocost = 1000.0  # will make the cost very large
        return ocost
        
    objCosts = [ calcObjsCost([x, y]) ]
    # check for obstacles along the way  We assume no curvature
    displacementUnit = vector_normalize([x,y])
    orig_length_squared = vector_length_squared( [x, y])
    step = .1
    displacementLength = step
    while displacementLength * displacementLength - .001 < orig_length_squared:
        displacementVec = vector_scale(displacementUnit, displacementLength)
        displacementLength += step
        ocost = calcObjsCost(displacementVec)
        objCosts = objCosts + [ocost]

    # average trajectory object collision costs
    ocost = sum(objCosts) / float(len(objCosts))
    
    obsCoefficient = 3.0  # Assign some multipliers for the costs
    goalCoefficient = 3.0
    speedCoefficient = -1.0
    #obsCost = g.distToNearestObstacle(x-oldX, y-oldY)
    speed = v
    goalDistance = vector_length(vector_minus(goal, [x,y]))

    ocost = obsCoefficient * ocost
    gcost = goalCoefficient*goalDistance
    scost = speedCoefficient*speed

    if goalDistance < .5:
        scost = 0 # speed shall not contribute to the cost when we are really close to the goal

    #if obsDistance < 1.0:  # too close to an obstacle -- invalidate this option
        #ocost += (1.0 - obsDistance)   # will make the cost very large
    
    cost =  ocost + gcost +  scost

   
    if not name:
        name = "%i %i" % (int(math.floor(x*3)), int(math.floor(y*3)))
    name = "cost %s" % name
    viz.vizSegment([x, y, 0.0], [x, y, cost / 60.0 + .03 ], name=name)#, color=(1.0, 0.0, 0.0))
    rospy.loginfo("(%0.2f, %0.2f)  obs = %0.2f  goal = %0.2f  speed = %0.2f  total = %0.2f", x, y, ocost, goalDistance, speed, cost)
    return [cost, ocost, gcost, scost]

class OccupancyGridCell:
    def __init__(self):
        self.distanceToObstacle = 0.0
        self.collidesWithObstacle = None
        self.distanceToGoal = 10.0
        
class OccupancyGrid:
    def __init__(self, li, viz):
        self.li = li # laser interpreter
        self.minX = 0.0
        self.maxX = 2.5
        self.minY = -2.5
        self.maxY = 2.5
        self.bucketsPerDimension = 15
        self.viz = viz
        self.grid = [OccupancyGridCell() for x in range(0, self.bucketsPerDimension * self.bucketsPerDimension)]
        self._bucketCenters = None
        self.goal = [0, 0]
        
    #givena  tuple containing floats, returns the value in the occupancy grid 
    def getGridValue(self, point):
        [x, y] = self.pointToBucketXY(point)
        return self.getBucketValue( x, y)
    
    def setGridValue(self, point, value):
        [x, y] = self.pointToBucketXY(point)
        self.setBucketValue(x, y, value)

    
    def setBucketValue(self, x, y, value):
        self.grid[x * self.bucketsPerDimension + y] = value
        
    def getBucketValue(self, x, y):
        return self.grid[x * self.bucketsPerDimension + y]

    def bucketCenters(self):
        if not self._bucketCenters:
            self._bucketCenters = [x for x in self.computeBucketCenters()]
        return self._bucketCenters

    def computeBucketCenters(self):
        stepX = self.calculateSpacing(True)
        stepY = self.calculateSpacing(False)
        for iX in range(0, self.bucketsPerDimension):
            fX = float(iX) * stepX + self.minX
            for iY in range(0, self.bucketsPerDimension):
                fY = float(iY) * stepY + self.minY
                if fX > 0:
                    yield [fX, fY, iX, iY]

    # returns integer [iX, iY] of the bucket that corresponds to the given floating point values                    
    def pointToBucketXY(self, pt):
        [x, y] = pt
        # if we are looking for -2 1 in a grid, we would return 
        xBucket = mapFloatIntoDiscretizedBucket(x, self.minX, self.maxX, self.bucketsPerDimension)
        yBucket = mapFloatIntoDiscretizedBucket(y, self.minY, self.maxY, self.bucketsPerDimension)
        return [xBucket, yBucket]

    # calculates the spacing in the given dimension (True for X , False for Y)
    def calculateSpacing(self, xdimp):
        if xdimp:
            return float(self.maxX - self.minX) / float(self.bucketsPerDimension)
        else:
            return float(self.maxY - self.minY) / float(self.bucketsPerDimension)

    # vizualizes the occupancy griz via rviz.  creates a line segment from the ground up
    # where the z is determined by the distance to the nearest obstacle
    def vizGrid(self):
        #rospy.loginfo("grid spacing => %0.2f", self.calculateSpacing())
        for [fX, fY, iX, iY] in self.bucketCenters():
            gridCell = self.getGridValue([fX, fY])
            #gridVal = gridCell.distanceToObstacle
            gridVal = self.obstacleCostAt(fX, fY)
            rospy.loginfo("viz grid [%0.2f, %0.2f] => %0.2f", fX, fY, gridCell.distanceToObstacle)
            self.viz.vizSegment([fX,fY,0.0], [fX,fY,gridVal * .2 ], name="forward %i %i" % (iX, iY), color=(0.0, .7, .2))

    def updateGridCollisions(self):
        # first update the collidsWithObstacle part of each gridcell using the laser readings
        for [fX, fY, iX, iY] in self.bucketCenters():
            if fX > 0:
                vec = [fX, fY]
                # decide whether there is something in or before the
                # grid square by casting a ray in that direction
                cast_distance = self.li.castVector(vec)
                fY = -1.0 * fY
                vec = [fX, fY]
                distance_to_fXfY_squared = vector_length_squared(vec)
                # 0 weight for 
                gridCell = self.getGridValue(vec)
                obstacle_in_grid_squarep = cast_distance * cast_distance < distance_to_fXfY_squared
                
                if obstacle_in_grid_squarep:
                    gridCell.distanceToObstacle = 0
                    gridCell.collidesWithObstacle = True
                else:
                    gridCell.distanceToObstacle = cast_distance - sqrt(distance_to_fXfY_squared)
                    gridCell.collidesWithObstacle = False
                #self.setGridValue(vec, cast_distance - sqrt(distance_to_fXfY_squared))

    def updateGridDistances(self, goal):
        self.goal = goal
        # update distanceToObstacle for each cell
        for [fX, fY, iX, iY] in self.bucketCenters():
            dist = self.computeDistToNearestObstacle(fX, fY)
            gridCell = self.getGridValue([fX, fY])
            gridCell.distanceToObstacle = dist
            gridCell.distanceToGoal = vector_length(vector_minus(goal, [fX, fY]))

    # update the grid with the laser scan info
    def updateGrid(self, goal):
        # first update the collidsWithObstacle part of each gridcell using the laser readings
        self.updateGridCollisions()
        # then update the distanceToObstacle part of each gridCell using the collision data
        self.updateGridDistances(goal)
        #visualize the grid
        self.vizGrid()

    # distToNearestObstacle:
    #   An occupancy grid function - probably want this in the class
    #   Does an inefficient search for the nearest obstance in the grid
    def distToNearestObstacle(self, startX, startY):
        gridCell = self.getGridValue([startX, startY])
        return gridCell.distanceToObstacle

    def distToGoal(self, startX, startY):
        return vector_length(vector_minus([startX, startY], self.goal))
        gridCell = self.getGridValue([startX, startY])
        return gridCell.distanceToGoal

    def obstacleCostAt(self, x, y):
        obsDist = self.distToNearestObstacle(x, y)
        ocost = -obsDist
        if obsDist < 0.2:  # too close to an obstacle -- invalidate this option
            ocost = 1000.0  # will make the cost very large if we hit an obstacle
        return ocost
    
    def computeDistToNearestObstacle(self, startX, startY):
        minDist = 4.0
        # simply loops over all the buckets and find the one closest to us that is an obstacle
        # OMG! O(n^4)!  Nonetheless, for a 12x12 grid that is only 20,000 iterations.  Why program
        # an efficient algorithm when this will work for our purposes?
        for [fX, fY, iX, iY] in self.bucketCenters():
            vec = [fX, fY]
            gridCell = self.getGridValue(vec)
            if gridCell.collidesWithObstacle:
                vToStart = vector_minus([startX, startY], vec)
                dist = vector_length(vToStart)  #abs(vec[0]-startX) + abs(vec[1]-startY)
                dist += -.343
                if dist < 0:
                    dist = 0
                if dist < minDist:
                    minDist = dist
        return minDist
