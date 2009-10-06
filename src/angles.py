#!/usr/bin/env python
import roslib; roslib.load_manifest('ros_hw1')
import rospy 
import tf	# needed to read odometry values
import math	# not sure how this is used
import copy # to do deepcopy
from sensor_msgs.msg import LaserScan	#reads laser readings
from geometry_msgs.msg import Twist 	#gives out movement commands
from geometry_msgs.msg import Vector3	#to specify the data for movement

#some useful #defs
r2d = 180.0/3.14159	# radians to degree conversion rate
X_SPEED = 0.2
SLOW_TURN_SPEED = 3.0/r2d
FAST_TURN_SPEED = 20.0/r2d
DISTANCE_BEFORE_TURN = 2.0

# LaserInterpreter class: is passed each laser reading, does some calculations
class LaserInterpreter:
  def __init__(self): # constructor
    self.readings = []
    self.maxReadings = 10
    
  def laserReadingNew(self, reading):
    # Do some processing on the new laser reading
    self.readings.append(reading)
    if len(self.readings) > self.maxReadings:
      self.readings.pop(0)	# remove the oldest reading
    #self.logReadingInfo(reading)
    
  def logReadingInfo(self, reading):
    rospy.loginfo("Min: %d  Max: %d  Inc: %f  Len: %d", r2d*reading.angle_min, r2d*reading.angle_max, r2d*reading.angle_increment, len(reading.ranges))
    print reading.ranges[-1], reading.ranges[len(reading.ranges)/2], reading.ranges[0]

#RobotPosition class : stores the current position of the robot, with respect to the initial position of the robot
class RobotPosition:
  def __init__(self):
    self.initialized = False
    self.trans = [0,0]
    self.rot = 0

  def initialPos(self, t, r):
    self.trans0 = t
    self.rot0 = r[2]
    rospy.loginfo ("initial values: %f, angle: %f",self.trans0[0],self.rot0)

  def positionNew(self, t, r):
    self.trans[0] = t[0] - self.trans0[0]
    self.trans[1] = t[1] - self.trans0[1]
    self.rot = r[2] - self.rot0
    #self.logPosInfo()

  def logPosInfo(self):
    rospy.loginfo("Odometry: (%0.2f, %0.2f) at %0.2f degrees", self.trans[0], self.trans[1], self.rot*r2d)

#VelocityCommander class: to set velocities for the robot
class VelocityCommander:
  def __init__(self):
    self.linearV = [0,0,0]
    self.angularV = [0,0,0]

#global variables
li = LaserInterpreter()	# global LaserInterpreter object
rp = RobotPosition() # global RobotPosition object

#local callback function for the readings passed in by roscore
def callback(reading):
	li.laserReadingNew(reading)
	
	
def DoNaiveOdometry(rp):
	# this part just tries a naive odometry-based turn.. does not work
	#create local variables
	thetavel = 0.0
	xvel = 0.0

	if rp.trans[0] < DISTANCE_BEFORE_TURN:	
	  # not yet travelled 10 m - move forward 
	  xvel = X_SPEED
	elif rp.trans[0] >=  DISTANCE_BEFORE_TURN and rp.rot < 150.0/r2d:	# travelled 10m - start turning quickly
	  thetavel = FAST_TURN_SPEED
	elif rp.trans[0] >=  DISTANCE_BEFORE_TURN and rp.rot < 170.0/r2d:	# getting close - slow down the turning
	  thetavel = SLOW_TURN_SPEED
	elif rp.trans[0] >=  DISTANCE_BEFORE_TURN and rp.rot >= 180.0/r2d:	# travelled 10m and turned around - move forward again
	  xvel = X_SPEED 

	return xvel,thetavel


#function to check if the laser reading is the same as when we started, only inverted
def checkLaserReading(initialLasers):
	#print initialLasers
	'''
	if initialLasers == li.readings[-1]:
		return True
	else:
		return False'''
	count = 1
	for i in initialLasers:
		# compare against the most recent reading
		if (i != li.readings[-1][count]):
			print "Nope.. didn't match at index:"%count
			return False
		count = count+1
	return True

''' 
method 1: take laser readings before turning, then compare the readings at every point .
If they match, infer that we have returned to orig. pos.
'''
def DoMethodOne(rp):
	#some bools to keep sanity
	startTurning = False
	done = False
	goingBack = False
	#create local variables
	thetavel = 0.0
	xvel = 0.0
	#method 1: try to align using laser reading
	if rp.trans[0] <  DISTANCE_BEFORE_TURN:	
		# not yet travelled 10 m - move forward 
		xvel = X_SPEED
	elif rp.trans[0] >=  DISTANCE_BEFORE_TURN:   #start/continue the turn 
		#take initial laser reading only the first time   
		if startTurning == False:              
			#store the most recent laser reading as the initial reading. Do a deep copy to prevent loss
			initial_lasers = copy.deepcopy(li.readings[-1]) 
			startTurning = True
			xvel = 0.0
			thetavel = SLOW_TURN_SPEED
		else:
			#see if the laser readings are the same
			if not goingBack:
		  		done =	checkLaserReading(initial_lasers)
		  	else:
		  		done = True		  		
	  	if done:
	  		#we have returned to our initial position.. go straight back now
	  		#dont check laser readings anymore
	  		goingBack = True
	  		xvel = X_SPEED
	  		thetavel = 0.0
	  	else:
	  		#not yet done turning.. keep turning
	  		xvel = 0.0
	  		thetavel = SLOW_TURN_SPEED
	return xvel,thetavel
	
# main method of the program
def angles():
	#initialize the node and subscribe to Laser and odom
	rospy.init_node('angles')
	rospy.loginfo('"Angles" node is awake')
	rospy.Subscriber("laser", LaserScan, callback) # listen to "laser"
	rospy.loginfo("Subscribed to laser readings")
	odoListener = tf.TransformListener() # listen to tf
	rospy.loginfo("Subscribed to odometry frames")
	rate = rospy.Rate(10.0) # 10 Hz
	velPublish = rospy.Publisher("commands", Twist) # publish to "commands"
	#initial odometry lookup
	while not rp.initialized:
		try:
		  (trans, rot) = odoListener.lookupTransform('/odom', '/base_link', rospy.Time(0))
		  rp.initialized = True
		except (tf.LookupException, tf.ConnectivityException):
		  continue
	rp.initialPos(trans, rot)


	#keep reading odometry every 1/10s
	while not rospy.is_shutdown():
		try:
		  (trans, rot) = odoListener.lookupTransform('/odom', '/base_link', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException):
		  continue
		rp.positionNew(trans, rot)
		
		#move the robot now
		#choose the method for sensor interpretation
		xvel,thetavel = DoMethodOne(rp)
		#send the command
		twist = Twist(Vector3(xvel,0,0),Vector3(0,0,thetavel))
		velPublish.publish(twist)
		rate.sleep()
    

if __name__ == '__main__':
  try:
    angles()
  except rospy.ROSInterruptException:
    pass
