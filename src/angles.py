#!/usr/bin/env python
import roslib; roslib.load_manifest('hw1')
import rospy
import tf
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

r2d = 180.0/3.14159

class LaserInterpreter:
  def __init__(self): # constructor
    self.readings = []
    self.maxReadings = 10
  def laserReadingNew(self, reading):
    # Do some processing on the new laser reading
    self.readings.append(reading)
    if len(self.readings) > self.maxReadings:
      self.readings.pop(0)	# remove the oldest reading
    self.logReadingInfo(reading)
  def logReadingInfo(self, reading):
    rospy.loginfo("Min: %d  Max: %d  Inc: %f  Len: %d", r2d*reading.angle_min, r2d*reading.angle_max, r2d*reading.angle_increment, len(reading.ranges))
    print reading.ranges[-1], reading.ranges[len(reading.ranges)/2], reading.ranges[0]

class RobotPosition:
  def __init__(self):
    self.initialized = False
    self.trans = [0,0]
    self.rot = 0
  def initialPos(self, t, r):
    self.trans0 = t
    self.rot0 = r[2]
  def positionNew(self, t, r):
    self.trans[0] = t[0] - self.trans0[0]
    self.trans[1] = t[1] - self.trans0[1]
    self.rot = r[2] - self.rot0
    self.logPosInfo()
  def logPosInfo(self):
    rospy.loginfo("Odometry: (%0.2f, %0.2f) at %0.2f degrees", self.trans[0], self.trans[1], self.rot*r2d)

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
  velPublish = rospy.Publisher("commands", Twist) # publish to "commands"
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
    rp.positionNew(trans, rot)
    rate.sleep()

if __name__ == '__main__':
  try:
    angles()
  except rospy.ROSInterruptException:
    pass
