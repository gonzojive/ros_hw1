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
from roslib.rostime import Duration

def mpublisher():
    return rospy.Publisher("visualization_marker", Marker)

def demo():
    pub = mpublisher()
    pub.publish(gsphere)
    

def gsphere():
    marker = Marker()
    marker.header.frame_id = "/base_laser";
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = Marker.SPHERE;
    marker.action = Marker.ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.lifetime.secs = 55
    return marker
