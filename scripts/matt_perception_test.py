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
from multiprocessing import Process
import sys


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

		#self.am_i_checking = False
		self.matt_pub_done = rospy.Publisher("point_done", done, queue_size = 10)
		#self.matt_pub_point3D = rospy.Publisher("point3D", Coordinates_3D, queue_size = 10)

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
		#self.done_class_point.done = False
		#if self.am_i_checking:
		self.done_class_point = done
		
		
        	#self.done_class_point.done = True
		#self.done_class_point = done()
		
        	print('received done callback saying:')
		print(done.done)

	def scan(self):
		i= 0
		#self.done_class_point.done = False
		
		#self.am_i_checking = True

		# publish that no point has been found - this will be replaced by a new message if YOLO detects something
		
		#self.matt_pub_point3D.publish(self.done_class_point)
		print("Waiting to settle")

		###
		uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        	roslaunch.configure_logging(uuid)

        	self.yolo_launch = roslaunch.parent.ROSLaunchParent(uuid, ['/home/matt/tiago_public_ws/src/object_detection/launch/yolo.launch'])

		self.yolo_launch.start()


		rospy.sleep(2)
		print("Publishing no point found and waiting")
		self.done_class_point.done = False
		self.matt_pub_done.publish(self.done_class_point)
		rospy.sleep(2)
		print('Starting Scan')
		while i<2000:
			self.point_3d_client()
			
			if self.done_class_point.done:
				print("Found class")
                		self.done_class_point.done = False
				#self.am_i_checking = False
				self.yolo_launch.shutdown()
                		return True
				break
                	i += 1
		else:
			#self.am_i_checking = False
			self.yolo_launch.shutdown()
                	return False
		

	def execute(self,userdata):
	
		self.play_m_as = SimpleActionClient('/play_motion', PlayMotionAction)
		if not self.play_m_as.wait_for_server(rospy.Duration(20)):
			rospy.logerr("Could not connect to /play_motion AS")
			exit()
		rospy.loginfo("Connected!")

		pmg = PlayMotionGoal()
               	pmg.motion_name = 'home'#'pick_final_pose'
		pmg.skip_planning = False
		self.play_m_as.send_goal_and_wait(pmg)

		#uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        	#roslaunch.configure_logging(uuid)
        	#self.launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/matt/tiago_public_ws/src/darknet_ros/darknet_ros/launch/darknet_ros.launch"])
		#self.launch.start()
		print("EXECUTING")
		
		points = [[-0.5,-0.5,1],[-0.5,-0.5,0],[-0.5,6,0],[-0.5,6,-1],[6.5,7,-1]]
		index = 0
		while True:
			print("Calling scan function")
			result = self.scan()
			if result:
				print("Scan found an object")
				break
			else:
				print("Scan couldn't find anything")
				if index > len(points)-1:
					index = 0
				locations = points[index]
				print("MOVING TO OBSERVATION POINT")
				print(locations)
				move_base_global.runMoveBase(locations[0],locations[1],locations[2])
				index += 1
					
	
  
        	#self.launch.shutdown()

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
		smach.State.__init__(self, outcomes=['Picked','Failed'])

	def pick_client(self):
		rospy.wait_for_service('pick_gui')
		try:
			print("Trying pick client")
            		pickUp = rospy.ServiceProxy('pick_gui', Trigger)
			trigRequest = TriggerRequest()
            		res = pickUp(trigRequest)
			print(res)
			return res
			#print("Finished trying")
        	except rospy.ServiceException, e:
            		print "Service call failed: %s" % e

	def execute(self, userdata):

		print("Launching pick up")
		uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        	roslaunch.configure_logging(uuid)

        	self.launch2 = roslaunch.parent.ROSLaunchParent(uuid, ['/home/matt/tiago_public_ws/src/matt_test_package/launch/pick_demo.launch'])

		self.launch2.start()
		print("Starting pick client")
		result = self.pick_client()



		if result.success == True:
			print("SUCCEEDED PICK")
			self.launch2.shutdown()
			return 'Picked'
		else:
			print("RETRY")
			self.launch2.shutdown()
			return 'Failed'
        	



class navigateToBinClass(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['ReachedBin'])

	def execute(self, userdata):

		# TODO - ENTER COORDINATES OF BIN AND RELEASE OBJECT
		move_base_global.runMoveBase(7,0,-1)

		self.play_m_as = SimpleActionClient('/play_motion', PlayMotionAction)
		if not self.play_m_as.wait_for_server(rospy.Duration(20)):
			rospy.logerr("Could not connect to /play_motion AS")
			exit()
		rospy.loginfo("Connected!")

		pmg = PlayMotionGoal()
               	pmg.motion_name = 'pregrasp_weight'#'pick_final_pose'
		pmg.skip_planning = False
		self.play_m_as.send_goal_and_wait(pmg)

		pmg = PlayMotionGoal()
               	pmg.motion_name = 'open'#'pick_final_pose'
		pmg.skip_planning = False
		self.play_m_as.send_goal_and_wait(pmg)

        	return 'ReachedBin'



	
