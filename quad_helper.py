#!/usr/bin/python

import numpy as np
import math
import vrep
import PID
import time
import tf.transformations


class quad_helper(object):
	def __init__(self, clientID):
		
		
		
		self.clientID = clientID
		self.quadHandle = None
		self.pos = [0, 0, 0]
		self.rotor_data = [0.0, 0.0, 0.0, 0.0]
		self.orig_location = [0, 0, 0]
		self.curr_location = [0, 0, 0]
		self.target_z = 0.0

		'''
		Initializing angular rate PID controller
		'''
		#Tuned by plotting output graphs in vrep
		self.P_roll_rate = 0.1
		self.P_pitch_rate = 0.1
		self.P_yaw_rate = 0.002
		self.D_roll_rate = 0.001
		self.D_pitch_rate = 0.001
		self.D_yaw_rate = 0

		self.roll_rate_pid = PID.PID(self.P_roll_rate, 0, self.D_roll_rate)
		self.pitch_rate_pid = PID.PID(self.P_pitch_rate, 0, self.D_pitch_rate)
		self.yaw_rate_pid = PID.PID(self.P_yaw_rate, 0, self.D_yaw_rate)

		self.roll_rate_pid.SetPoint = 0
		self.pitch_rate_pid.SetPoint = 0
		self.yaw_rate_pid.SetPoint = 0


		'''
		Initializing attitude PID controller
		'''
		#Tuned by plotting output graphs in vrep
		self.P_roll = 0.6
		self.P_pitch = 0.6
		self.P_yaw = 0.25
		self.P_height = 0.2
		self.D_roll = 0.1
		self.D_pitch = 0.1
		self.D_yaw = 0
		self.D_height = 0.1

		self.roll_pid = PID.PID(self.P_roll, 0, self.D_roll)
		self.pitch_pid = PID.PID(self.P_pitch, 0, self.D_pitch)
		self.yaw_pid = PID.PID(self.P_yaw, 0, self.D_yaw)
		self.height_pid = PID.PID(self.P_height, 0, self.D_height)

		self.roll_pid.SetPoint = 0
		self.pitch_pid.SetPoint = 0
		self.yaw_pid.SetPoint = 0
		self.height_pid.SetPoint = 2

		'''
		Initializing position PID controller
		'''
		#Tuned by plotting output graphs in vrep
		self.P_x = 0.02
		self.P_y = 0.02
		self.I_x = 0.003
		self.I_y = 0.003
		self.D_x = 0.03
		self.D_y = 0.03

		self.x_pid = PID.PID(self.P_x, self.I_x, self.D_x)
		self.y_pid = PID.PID(self.P_y, self.I_y, self.D_y)

		self.x_pid.SetPoint = 0
		self.y_pid.SetPoint = 0
		
	'''
	Initialize all sensors and reset quadcopter position in world
	'''

	def init_sensors(self):
		# Initialize IMU
		err, self.quadHandle = vrep.simxGetObjectHandle(self.clientID, 'Quadricopter', vrep.simx_opmode_blocking)


		# Reset quadcopter position
		err, self.pos = vrep.simxGetObjectPosition(self.clientID, self.quadHandle, -1, vrep.simx_opmode_buffer)
		self.pos[2] = 1.0
		vrep.simxSetObjectPosition(self.clientID, self.quadHandle, -1, self.pos, vrep.simx_opmode_oneshot)
		err, self.orig_location = vrep.simxGetObjectPosition(self.clientID, self.quadHandle, -1, vrep.simx_opmode_buffer)

	'''
	Start V-REP simulation
	'''

	def start_sim(self):
		# vrep.simxStartSimulation(self.clientID, vrep.simx_opmode_oneshot_wait)
		vrep.simxSynchronous(self.clientID, True)
		dt = .001
		vrep.simxSetFloatingParameter(self.clientID,vrep.sim_floatparam_simulation_time_step,dt,vrep.simx_opmode_oneshot)
		vrep.simxStartSimulation(self.clientID,vrep.simx_opmode_oneshot);
		return

	'''
		Stop V-REP simulation
	'''

	def stop_sim(self):
		vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_oneshot_wait)
		return

	'''
	This function gets quadcopter state
	'''

	def get_state(self):
		self.pos = vrep_imu.get_pos(self.clientID, self.quadHandle)
		return self.pos

	'''
	This function gets quadcopter body frame orientation
	'''

	def get_orientation(self):
		err, self.quadHandle = vrep.simxGetObjectHandle(self.clientID, 'Quadricopter', vrep.simx_opmode_oneshot)

		# Reset quadcopter position
		err, self.orientation = vrep.simxGetObjectOrientation(self.clientID, self.quadHandle, -1, vrep.simx_opmode_oneshot)
		return self.orientation


	'''
	This function gets quadcopter body frame position
	'''

	def get_position(self):
		err, self.quadHandle = vrep.simxGetObjectHandle(self.clientID, 'Quadricopter', vrep.simx_opmode_oneshot)

		# Reset quadcopter position
		err, orientation = vrep.simxGetObjectPosition(self.clientID, self.quadHandle, -1, vrep.simx_opmode_oneshot)
		return orientation

	'''
	This function gets quadcopter angular rates
	'''

	def get_ang_rates(self):
		err, self.quadHandle = vrep.simxGetObjectHandle(self.clientID, 'Quadricopter', vrep.simx_opmode_oneshot)

		# Reset quadcopter angular rates
		err, temp, ang_rates = vrep.simxGetObjectVelocity(self.clientID, self.quadHandle, vrep.simx_opmode_oneshot)
		return ang_rates

	'''
	This function updates the angular rate PID controller
	'''

	def update_ang_rate_pid(self):
		
		ang_rates = self.get_ang_rates()
		

		roll_rate_fb = ang_rates[0]
		pitch_rate_fb = ang_rates[1]
		yaw_rate_fb = ang_rates[2]

		self.roll_rate_pid.update(roll_rate_fb)  
		self.pitch_rate_pid.update(pitch_rate_fb)  
		self.yaw_rate_pid.update(yaw_rate_fb)

		roll_rate_output = self.roll_rate_pid.output
		pitch_rate_output = self.pitch_rate_pid.output
		yaw_rate_output = self.yaw_rate_pid.output

		ang_acc = [roll_rate_output,pitch_rate_output,yaw_rate_output]

		return ang_acc

	'''
	This function updates the attitude PID controller
	'''

	def update_att_pid(self):
		att = self.get_orientation()
		pos = self.get_position()

		height_fb = pos[2]
		roll_fb = att[0]
		pitch_fb = att[1]
		yaw_fb = att[2]

		self.roll_pid.update(roll_fb)  
		self.pitch_pid.update(pitch_fb)  
		self.yaw_pid.update(yaw_fb)
		self.height_pid.update(height_fb)

		roll_output = self.roll_pid.output
		pitch_output = self.pitch_pid.output
		yaw_output = self.yaw_pid.output
		height_output = self.height_pid.output

		height_output = height_output/(math.cos(roll_fb)*math.cos(pitch_fb))

		self.roll_rate_pid.SetPoint = roll_output
		self.pitch_rate_pid.SetPoint = pitch_output
		self.yaw_rate_pid.SetPoint = yaw_output

		return [height_output]

	'''
	This function updates the position PID controller
	'''

	def update_pos_pid(self):
		pos = self.get_position()
		
		x_fb = pos[0]
		y_fb = pos[1]

		self.x_pid.update(x_fb)  
		self.y_pid.update(y_fb)  

		x_output = self.x_pid.output
		y_output = self.y_pid.output

		self.roll_pid.SetPoint =  -y_output
		self.pitch_pid.SetPoint = x_output

	'''
	This function sets a the quadcopter angular acceleration
	'''

	def set_ang_acc(self,ang_acc):
		mass_norm_acc = float(ang_acc[0])
		roll_ang_acc = float(ang_acc[1])
		pitch_ang_acc = float(ang_acc[2])
		yaw_ang_acc = float(ang_acc[3])

		print(mass_norm_acc)

		# print(roll_ang_acc)
		# print(pitch_ang_acc)
		# print(yaw_ang_acc)

		vrep.simxSetFloatSignal(self.clientID,'mass_norm_acc', mass_norm_acc,vrep.simx_opmode_oneshot)
		vrep.simxSetFloatSignal(self.clientID,'roll_ang_acc', roll_ang_acc,vrep.simx_opmode_oneshot)
		vrep.simxSetFloatSignal(self.clientID,'pitch_ang_acc', pitch_ang_acc,vrep.simx_opmode_oneshot)
		vrep.simxSetFloatSignal(self.clientID,'yaw_ang_acc', yaw_ang_acc,vrep.simx_opmode_oneshot)
		return


	'''
	This function sets the quadcopter position setpoint
	'''
	def move_quad(self,pos):
		# Initialize IMU
		err, self.quadTargetHandle = vrep.simxGetObjectHandle(self.clientID, 'Quadricopter_target', vrep.simx_opmode_blocking)
		# Sets quadcopter position
		vrep.simxSetObjectPosition(self.clientID, self.quadTargetHandle, -1, pos, vrep.simx_opmode_oneshot)

	'''
	This function gets the object position
	'''
	def get_obj_pos(self):
		# Initialize IMU
		err, objHandle = vrep.simxGetObjectHandle(self.clientID, 'Object', vrep.simx_opmode_oneshot)
		# Gets the object position
		err, obj_pos = vrep.simxGetObjectPosition(self.clientID, objHandle, -1, vrep.simx_opmode_oneshot)
		# print(obj_pos)
		return obj_pos
	'''
	This function gets the object orientation
	'''
	def get_obj_orien(self):
		# Initialize IMU
		err, objHandle = vrep.simxGetObjectHandle(self.clientID, 'ObjectFrame', vrep.simx_opmode_oneshot)
		err, quadHandle = vrep.simxGetObjectHandle(self.clientID, 'joint1', vrep.simx_opmode_oneshot)
		# Gets the object orientation
		err, obj_orien = vrep.simxGetObjectQuaternion(self.clientID, objHandle,quadHandle, vrep.simx_opmode_oneshot)
		obj_orien_rot = tf.transformations.quaternion_matrix([obj_orien[0], obj_orien[1], obj_orien[2],obj_orien[3]])
		# print(obj_orien)
		return obj_orien_rot

	'''
	This function calculates Inverse Kinematics
	'''
	def get_joint_pos(self,obj_orien_rot):
		d1 = 0;
		# theta1 = math.degrees(math.acos(obj_orien_rot[2,2]));
		# theta2 = math.degrees(math.acos(obj_orien_rot[1,1]));
		# print(theta1)
		# print(theta2)
		# theta1 = math.acos(obj_orien_rot[2,2]);
		# theta2 = math.acos(obj_orien_rot[1,1]);
		theta1 = math.atan2(obj_orien_rot[0,2],obj_orien_rot[2,2]);
		theta2 = math.atan2(obj_orien_rot[1,0],obj_orien_rot[1,1]);
		return [d1,theta1,theta2]

	'''
	This function sets the joint positions
	'''
	def set_joint_pos(self,joint_pos):
		# print(joint_pos)
		temp = self.clientID
		ret1, joint1 = vrep.simxGetObjectHandle(self.clientID, 'joint1',vrep.simx_opmode_oneshot)
		ret1, joint2 = vrep.simxGetObjectHandle(self.clientID, 'joint2',vrep.simx_opmode_oneshot)
		ret1, joint3 = vrep.simxGetObjectHandle(self.clientID, 'joint3',vrep.simx_opmode_oneshot)
		vrep.simxSetJointPosition(self.clientID, joint1,joint_pos[0],vrep.simx_opmode_oneshot)
		vrep.simxSetJointPosition(self.clientID, joint2,joint_pos[1],vrep.simx_opmode_oneshot)
		vrep.simxSetJointPosition(self.clientID, joint3,joint_pos[2],vrep.simx_opmode_oneshot)
