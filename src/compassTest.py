#!/usr/bin/env python
import roslib; roslib.load_manifest('hw1')
import rospy
import tf
from math import *
from line import *
from lineviz import *
from compass import *
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
  def __init__(self, p, c): # constructor
    self.compass = c
#    self.position = p
    self.doRansac = 1
    self.mapviz = LocalMapVisualizer()

  def laserReadingNew(self, reading):
    rospy.loginfo('Laser reading received...')  
    # Do some processing on the new laser reading
    if self.doRansac >= 1:
      self.ransac(reading)

  def logReadingInfo(self, reading):
    rospy.loginfo("Min: %d  Max: %d  Inc: %f  Len: %d", r2d*reading.angle_min, r2d*reading.angle_max, r2d*reading.angle_increment, len(reading.ranges))
    print reading.ranges[-1], reading.ranges[len(reading.ranges)/2], reading.ranges[0]

  def ransac(self, reading):
    cartesianPoints = laserReadingToCartesianPoints(reading)
    for pt in cartesianPoints:
      rospy.loginfo("(%0.2f, %0.2f)", pt[0], pt[1])
    [bestLine, inliers] = fitLineWithRansac(cartesianPoints, .03)
    self.mapviz.vizPoints(inliers)
    def s(arr):
      return "[%0.2f, %0.2f]" % (arr[0], arr[1])
    rospy.loginfo("Line: %s trajectory: %s" % (s(bestLine.origin),  s(bestLine.trajectory)))
#    rospy.loginfo("  %i Inliers of %i readings: %s" % (len(inliers), len(reading.ranges), map(s, inliers)))
    angle = self.compass.getOrientation(bestLine)
    rospy.loginfo("Compass reading: MasterD = (%0.2f, %0.2f)  Angle = %0.2f", self.compass.master.trajectory[0], self.compass.master.trajectory[1], angle*r2d)


class RobotPosition:
  def __init__(self):
    self.initialized = False  # can't start until we initialize odometry
    self.odomTrans = [0,0]  
    self.odomRot = 0
    self.offsetTrans = [0,0]
    self.offsetRot = 0
  def resetOdom(self, t, r):  # resets the odometry offsets
    self.odomTrans0 = t
    self.odomRot0 = r[2]
  def odomReadingNew(self, t, r):  # calculate a new odometry reading with respect to the offsets
    self.odomTrans[0] = t[0] - self.odomTrans0[0]
    self.odomTrans[1] = t[1] - self.odomTrans0[1]
    self.odomRot = r[2] - self.odomRot0
  def compassReading(self, angles):  # take in a new compass reading
    realAngle = 0
    minDiff = 1000
    for a in angles:
      if abs(self.rotation()-a) < minDiff:  # closest angle so far
        realAngle = a
    self.offsetRot = realAngle - self.rotation()  # make the offset take it to the real angle
  def position(self):  # returns the position
    return [self.offsetTrans[0]+self.odomTrans[0], self.offsetTrans[1]+self.odomTrans[1]]
  def rotation(self):  # returns the rotation
    return self.offsetRot + self.odomRot
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
    self.velPublish.publish(Twist(Vector3(0,0,0),Vector3(0,0,4.0/r2d)))
    
compass = Compass()
rp = RobotPosition() # global RobotPosition object
li = LaserInterpreter(rp, compass)	# global LaserInterpreter object


def callback(reading):
  li.laserReadingNew(reading)

def init_node():
  rospy.init_node('kludge1_1')
  rospy.loginfo('"KLUDGE 1.1" node is awake')
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
  rp.resetOdom(trans, rot)
  while not rospy.is_shutdown():
    try:
      (trans, rot) = odoListener.lookupTransform('/odom', '/base_link', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException):
      continue
#    rospy.loginfo("Odometry: (%0.2f, %0.2f) at %0.2f", trans[0], trans[1], rot[2])
    rp.odomReadingNew(trans, rot)
    if compass.initialized:
      cmd.send()
    rate.sleep()

if __name__ == '__main__':
  try:
    init_node()
  except rospy.ROSInterruptException:
    pass
