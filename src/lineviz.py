import roslib; roslib.load_manifest('hw1')
import rospy
import tf
import visualization_msgs
import visualization_msgs.msg
import roslib.message
import roslib.rostime
import geometry_msgs.msg
import roslib.msg
import std_msgs.msg

from visualization_msgs.msg import *
from geometry_msgs.msg import *
from roslib.rostime import Duration

LINE_WIDTH = .02
POINT_WIDTH = .04

class LocalMapVisualizer:
    def __init__(self):
        self.pub = rospy.Publisher("visualization_marker", Marker)
        self.idCounter = 100
    def vizSegment(self, start, end, the_id=None):
        marker = Marker()
        marker.header.frame_id = "/base_laser";
        marker.ns = "basic_shapes";
        if the_id:
            marker.id = the_id
        else:
            marker.id = self.idCounter;
        self.idCounter = self.idCounter + 1
        marker.type = Marker.LINE_LIST;
        marker.action = Marker.ADD;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = LINE_WIDTH;
        marker.scale.y = LINE_WIDTH;
        marker.scale.z = LINE_WIDTH;
        if the_id == 1:
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
        elif the_id == 2:
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker.color.a = 1.0;
        else:
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
        
        marker.lifetime.secs = .100
        marker.points = map (lambda pt : Point(x = pt[0], y = pt[1]), [start, end])
        self.pub.publish(marker)

    def vizPoints(self, points, the_id=None):
        marker = Marker()
        marker.header.frame_id = "/base_laser";
        marker.ns = "basic_shapes";
        if the_id:
            marker.id = the_id
        else:
            marker.id = self.idCounter;
        self.idCounter = self.idCounter + 1
        marker.type = Marker.POINTS;
        marker.action = Marker.ADD;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = POINT_WIDTH;
        marker.scale.y = POINT_WIDTH;
        marker.scale.z = POINT_WIDTH;
        marker.color.r = 1.0;
        marker.color.g = .40;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.lifetime.secs = 700.100
        marker.points = map (lambda pt : Point(x = pt[0], y = pt[1]), points)
        self.pub.publish(marker)

