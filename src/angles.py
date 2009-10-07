#!/usr/bin/env python
import roslib; roslib.load_manifest('hw1')
import rospy
import tf
from math import *
from ransac import *
# import cv	# doesn't recognize cv
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

r2d = 180.0/3.14159

#Given an angle and a radius, returns the X,Y coordinates of that point in cartesian coordinates
# this assumes the following:
# 1.  The cartesian coordinate is such that in front of the robot is positive X and the LEFT of
# the robot is positive Y
# 2.  theta = 0, r = 1 corresponds to (0, -1) and theta = pi/2, r=1 corresponds to (1, 0)
def polarToCartesian(r, theta):
  x = r * cos(theta)
  y = r * sin(theta)
  return [y, -x]

# Given a bunch of laser readings
def laserReadingAngle(i, readingRanges):
  num_scan_points = len(readingRanges)
  if num_scan_points > 0:
    return (1.0 - float(i) / float(num_scan_points)) * pi
  else:
    return 0 #degenerate

def laserReadingToCartesianPoints(reading):
  return map(lambda rng,i: polarToCartesian(rng, laserReadingAngle(i, reading.ranges)), reading.ranges, xrange(0, len(reading.ranges)))

class LaserInterpreter:
  def __init__(self): # constructor
    self.readings = []
    self.maxReadings = 10
    self.R = 100  # number of subdivisions of radius
    self.T = 100  # number of subdivisions of theta
    self.maxRadius = 11.0  # laser only accurate to 12 feet, let's say 11 to be safe
    self.maxTheta = 3.14159  # theta in [0, pi]
    self.radiusInc = self.maxRadius / float(self.R)  # step size of radius
    self.thetaInc = self.maxTheta / float(self.T)  # step size of theta
    self.radiusValues = []  # start with empty arrays
    self.thetaValues = []
    self.doHough = 0  # determines whether to perform Hough or not
    self.doRansac = 1
    cur = 0.0
    for i in range(self.R):  # fill in the radius bins
      self.radiusValues.append(cur)
      cur += self.radiusInc
    cur = 0.0
    for i in range(self.T):  # fill in the theta bins
      self.thetaValues.append(cur)
      cur += self.thetaInc
    self.A = [self.T*[0] for i in range(self.R)]  # make the R x T accumulator array
  def laserReadingNew(self, reading):
    # Do some processing on the new laser reading
    self.readings.append(reading)
    if len(self.readings) > self.maxReadings:
      self.readings.pop(0)	# remove the oldest reading
    if self.doHough >= 1:
      self.hough(reading)
      self.doHough = 0
    if self.doRansac >= 1:
      self.ransac(reading)
      self.doRansac = 1
#    self.logReadingInfo(reading)
  def logReadingInfo(self, reading):
    rospy.loginfo("Min: %d  Max: %d  Inc: %f  Len: %d", r2d*reading.angle_min, r2d*reading.angle_max, r2d*reading.angle_increment, len(reading.ranges))
    print reading.ranges[-1], reading.ranges[len(reading.ranges)/2], reading.ranges[0]
  def hough(self, reading):
    curAngle = reading.angle_max  # I believe laser index 0 is the maximum angle, so we start there
    x = y = 0
    for dist in reading.ranges:  # cycle through the readings
      if dist >= self.maxRadius:  # if distance is too large, laser doesn't see anything
        continue
      x = dist*cos(curAngle)  # convert from polar to cartesian coords - this needs work, I don't think it's correct
      y = dist*sin(curAngle)
      rospy.loginfo("Reading found. Polar=(%0.2f,%0.2f degrees) --> Cartesian=(%0.2f,%0.2f)", dist, curAngle*r2d, x, y)
      for h in range(self.T):  # cycle through all bins of theta
        p = abs(x*cos(self.thetaValues[h]) + y*sin(self.thetaValues[h]))  # calculate the distance for the given theta
        k = int(round(p/self.radiusInc))  # find the bin for that distance
        if k == self.R:  # correct it if the bin was too far
          k -= 1
        if k == 0:  # this is the error case that's happening, k == 0 is getting a lot of hits
          rospy.loginfo("k = 0:  x=%0.2f, y=%0.2f, h=%0.2f, p=%0.2f", x, y, h, p)
        self.A[k][h] += 1  # increment the accumulator for the calculated distance, radius bin
      curAngle -= reading.angle_increment  # decrement the laser angle for the next reading
    peaks = []  # the list of local maxima in the parameter space
    numPeaks = 6  # arbitrarily chosen
    for distIndex in range(len(self.A)):  # cycle through the distances
      for angIndex in range(len(self.A[distIndex])):  # cycle through the angles
        if len(peaks) < numPeaks:  # if we have less than 6 peaks, just grab this one
          peaks.append([self.A[distIndex][angIndex], distIndex, angIndex])
          peaks.sort()
        elif self.A[distIndex][angIndex] > peaks[0]:  # otherwise, if this is greater than the current minimum, replace it
          peaks[0] = [self.A[distIndex, angIndex], distIndex, angIndex]  # [0] is minimum because we sort() every time
          peaks.sort()
    for peak in peaks:
      rospy.loginfo( "Count = %d  dist = %0.2f  angle = %0.2f", peak[0], self.radiusValues[peak[1]], self.thetaValues[peak[2]]*r2d)
  def ransac(self, reading):
    cartesianPoints = laserReadingToCartesianPoints(reading)
    [bestLine, inliers] = fitLineWithRansac(cartesianPoints, .03)
    #for pt in cartesianPoints:
    #rospy.loginfo("(%0.2f, %0.2f)", pt[0], pt[1])
    def s(arr):
      return "[%0.2f, %0.2f]" % (arr[0], arr[1])
    rospy.loginfo("Line: %s trajectory: %s" % (s(bestLine.origin),  s(bestLine.trajectory)))
#    rospy.loginfo("   Inliers: %s" % inliers)

class RobotPosition:
  def __init__(self):
    self.initialized = False
    self.trans = [0,0]
    self.rot = 0
  def initialPos(self, t, r):
    self.trans0 = t
    self.rot0 = r[2]
#    rospy.loginfo("%0.2f, %0.2f, %0.2f", self.trans0[0], self.trans0[1], self.rot0)
  def positionNew(self, t, r):
    self.trans[0] = t[0] - self.trans0[0]
    self.trans[1] = t[1] - self.trans0[1]
    self.rot = (r[2] - self.rot0) * 180
#    self.logPosInfo()
  def logPosInfo(self):
    rospy.loginfo("Odometry: (%0.2f, %0.2f) at %0.2f degrees", self.trans[0], self.trans[1], self.rot)

class Commands:
  def __init__(self, rp):
    self.xGoal = 4  # 4m
    self.thetaGoal = 180  # 180 degrees
    self.rp = rp  # the RobotPosition
    self.velPublish = rospy.Publisher("commands", Twist) # publish to "commands"
    self.stage = 1
  def send(self):
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

li = LaserInterpreter()	# global LaserInterpreter object
rp = RobotPosition() # global RobotPosition object

def callback(reading):
  li.laserReadingNew(reading)

def angles():
  rospy.init_node('angles')
  rospy.loginfo('"Angles" node is awake')
  rospy.Subscriber("laser", LaserScan, callback) # listen to "laser"
  rospy.loginfo("Subscribed to laser readings")
  odoListener = tf.TransformListener() # listen to tf
  rospy.loginfo("Subscribed to odometry frames")
  rate = rospy.Rate(10.0) # 10 Hz
  cmd = Commands(rp)
  while not rp.initialized:
    try:
      (trans, rot) = odoListener.lookupTransform('/odom', '/base_link', rospy.Time(0))
      rp.initialized = True
    except (tf.LookupException, tf.ConnectivityException):
      continue
  rp.initialPos(trans, rot)
  while not rospy.is_shutdown():
    try:
      (trans, rot) = odoListener.lookupTransform('/odom', '/base_link', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException):
      continue
#    rospy.loginfo("Odometry: (%0.2f, %0.2f) at %0.2f", trans[0], trans[1], rot[2])
    rp.positionNew(trans, rot)
#    cmd.send()
    rate.sleep()

if __name__ == '__main__':
  try:
    angles()
  except rospy.ROSInterruptException:
    pass
