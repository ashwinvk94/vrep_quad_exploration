#!/usr/bin/env python2

'''
ENPM 661 Spring 2019 
Final Project: Quadcopter Exploration

Authors:
Ashwin Varghese Kuruttukulam(ashwinvk94@gmail.com)
Rachith Prakash (rachithprakash@gmail.com)

Graduate Students in Robotics,
University of Maryland, College Park
'''

#!/usr/bin/env python2
'''
Airgilty

Updates RC commands to avoid collision with objects as detected by the obstacle_detector nodes 
If an obstacle is detected in the pitch axis, the quadcopter will move in the opposite direction
for a preset amount of time or until the obstacle is no longer in front. Whichever comes first.

Author:
Ashwin Varghese Kuruttukulam(ashwinvk94@gmail.com)
'''

import sys
import vrep
import PID
import rospy
import quad_helper
import tf.transformations
from geometry_msgs.msg import Pose
import numpy as np
import time

class pos_pub:
	def __init__(self):
		print 'started'
		# Init vrep client
		vrep.simxFinish(-1)
		clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
		if clientID != -1:
			#The quad helper contains function implementations for various quadcopter functions
			quad_functions = quad_helper.quad_helper(clientID)
			print('Main Script Started')
			quad_functions.init_sensors()
			quad_functions.start_sim()
			#Setting initial time
			init_time = time.time()
			d1=0

		#Initialize flags
		self.obstacleAvoidFlag = False

		# Rate init
		self.rate = rospy.Rate(20.0)  # MUST be more then 20Hz

		#Initializing publishers
		rospy.Subscriber("/quad_explore/target_position", Pose, self.posCb)
		# self.rcPub = rospy.Publisher("/mavros/rc/override",OverrideRCIn, queue_size = 10)

		while not rospy.is_shutdown() and vrep.simxGetConnectionId(clientID) != -1:
			#Getting object position with respect to first joint
			# obj_pos = quad_functions.get_obj_pos()
			# print 'Current Position'
			# print obj_pos

			#Setting the position of the quadcopter
			self.quad_pos = [0,0,3]
			print 'moving quad'
			quad_functions.move_quad(self.quad_pos)

			vrep.simxSynchronousTrigger(clientID);
			self.rate.sleep()
		quad_functions.stop_sim()
	
	'''
	Target Positon  callback
	'''
	def posCb(self,data):
		position = data.pose.position
		self.quad_pos = [position.x,position.y,position.z]

def main(args):
    rospy.init_node('pos_pub', anonymous=True)
    ic = pos_pub()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main(sys.argv)