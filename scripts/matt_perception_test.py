#!/usr/bin/env python
# # -*- coding: utf-8 -*-

import rospy
import smach
import numpy as np
import smach_ros
import time
from std_srvs.srv import Empty, Trigger, TriggerRequest
from final_msg_srv.srv import *
from final_msg_srv.msg import *
from std_msgs.msg import String
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry
import roslaunch
import roslaunch.parent
import move_base_test
import quat_to_euler
import move_base_global
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from actionlib import SimpleActionClient


print("Hello World!")


class matt_test_class(smach.State):
	def __init__(self):
        	print("This is the constructor method.")
	
		smach.State.__init__(self, outcomes=['FoundObjectTransition'], output_keys=['xco_out','yco_out'])

		self.done_message = False
        	rospy.Subscriber("point3D", Coordinates_3D, self.callback)
        	rospy.Subscriber("point_done", done, self.done_callback_1)
        	self.point3d_message = Coordinates_3D()
        	self.pub3d = rospy.Publisher("point3D_from_main", Coordinates_3D, queue_size=10)
        	self.done_class_point = done()

	def point_3d_client(self):
		rospy.wait_for_service('desired_object')
		#print("WAiting for 3d client service")
        	try:
			#print("Trying")
            		point3d = rospy.ServiceProxy('desired_object', Empty)
            		point3d()
			#print("Finished trying")
        	except rospy.ServiceException, e:
            		print "Service call failed: %s" % e

	def callback(self, Coordinates_3D):
        	self.point3d_message = Coordinates_3D
        	print('received point coordinates')


	def done_callback_1(self, done):
        	self.done_class_point.done = True
        	print('received done point')

	def execute(self,userdata):

		uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        	roslaunch.configure_logging(uuid)
        	self.launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/tiago_public_ws/src/darknet_ros/darknet_ros/launch/darknet_ros.launch"])
		print("EXECUTING")
		while True:
			#print("Checking")
            		self.point_3d_client()
			#print("Finished point3d client")
			#print(self.done_class_point.done)
            		if self.done_class_point.done:
				print("Found class")
                		self.done_class_point.done = False
                		break
        	self.launch.shutdown()

        	#self.saysomething_client("i got the 3-D point of the object, starting grasping")
		print("GOTIT")
        	print(self.point3d_message)
        	self.pub3d.publish(self.point3d_message)
		
		print("x coord")
		xco = self.point3d_message.x
		yco = self.point3d_message.y

		userdata.xco_out = xco
		userdata.yco_out = yco

        	return 'FoundObjectTransition'

class navClass(smach.State):
	def __init__(self):
		print("second class")
		smach.State.__init__(self, outcomes=['Navigated'], input_keys=['xco_in','yco_in'])
		#odom_sub = rospy.Subscriber('/mobile_base_controller/odom', Odometry, self.callback)
		#self.robot_x = 0
		#self.robot_y = 0
	

	#def callback(self, msg):
    		#self.robot_x = msg.pose.pose.position.x
		#self.robot_y = msg.pose.pose.position.y


		#angle = np.arctan(robot_y/robot_x)
		#quatx = msg.pose.pose.orientation.x
		#quaty = msg.pose.pose.orientation.y
		#quatz = msg.pose.pose.orientation.z
		#quatw = msg.pose.pose.orientation.w
		#print(quat_to_euler.quat_to_euler_func(quatx,quaty,quatz,quatw))
		
		#print(msg.pose.pose)

	def execute(self, userdata):
		
		xco = userdata.xco_in
		yco = userdata.yco_in
		move_base_test.runMoveBase(xco,yco)
        	return 'Navigated'



class pickClass(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['Picked'])

	def pick_client(self):
		rospy.wait_for_service('pick_gui')
		try:
			print("Trying pick client")
            		pickUp = rospy.ServiceProxy('pick_gui', Trigger)
			trigRequest = TriggerRequest()
            		res = pickUp(trigRequest)
			print(res)
			#print("Finished trying")
        	except rospy.ServiceException, e:
            		print "Service call failed: %s" % e

	def execute(self, userdata):
		print("Starting pick client")
		self.pick_client()
        	return 'Picked'



class navigateToBinClass(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['ReachedBin'])

	def execute(self, userdata):

		# TODO - ENTER COORDINATES OF BIN AND RELEASE OBJECT
		move_base_global.runMoveBase(7.5,-1)

		self.play_m_as = SimpleActionClient('/play_motion', PlayMotionAction)
		if not self.play_m_as.wait_for_server(rospy.Duration(20)):
			rospy.logerr("Could not connect to /play_motion AS")
			exit()
		rospy.loginfo("Connected!")

		pmg = PlayMotionGoal()
               	pmg.motion_name = 'offer'#'pick_final_pose'
		pmg.skip_planning = False
		self.play_m_as.send_goal_and_wait(pmg)

		pmg = PlayMotionGoal()
               	pmg.motion_name = 'open'#'pick_final_pose'
		pmg.skip_planning = False
		self.play_m_as.send_goal_and_wait(pmg)

        	return 'ReachedBin'



	
