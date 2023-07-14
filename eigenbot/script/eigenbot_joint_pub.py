#!/usr/bin/env python2

import numpy as np
import rospy
import rosnode
from sensor_msgs.msg import JointState
import matplotlib as plt
import math

simplified_mode = 1

def wrap_to_pi(angle):
    return np.remainder(angle + np.pi,  np.pi*2) - np.pi


class EigenbotJointPub():
	def __init__(self):
		self.initialized = False
		self.num_joints = 18
		self.joint_positions = [float('nan')]*self.num_joints
		self.joint_names = []
		self.initial_joint_positions = np.zeros(self.num_joints)
		self.rate = rospy.Rate(10)
		self.joint_cmd_pub = rospy.Publisher('/eigenbot/joint_cmd', JointState, queue_size=1)
		self.joint_fb_sub = rospy.Subscriber('/eigenbot/joint_fb', JointState, self.joint_fb_callback)
		self.imu = {}
    

	def main_loop(self):
		if simplified_mode == 1:
			print("Simple Model")
			# initialize constants
			omega = 300/(np.pi*2) #oscillation frequency in rad/s
			alpha = 1
			beta = 1
			lambda_cs = 1
			gait_a = np.pi/18 #from paper
			gait_b = np.pi/6 #from paper
			gait_n = 4 #from paper
			mew = 1 #from paper
			lambda_cs = 1
			ksum0 = np.zeros(6)
			K_array = np.array([[0, -1, -1, 1, 1, -1], [-1, 0, 1, -1, -1, 1], [-1, 1, 0, -1, -1, 1], [1, -1, -1, 0, 1, -1], [1, -1, -1, 1, 0, -1], [-1, 1, 1, -1, -1, 0]])
			t = 0
			dt = .001
			numlegs = 6
			cpg_x = [0.01] * numlegs #array sized 6 init to 0.01
			cpg_y = [0.01] * numlegs
			cpg_x[0]= -0.01
			cpg_y[0]= -0.01
			cpg_x[1]= 0.01
			cpg_y[1]= 0.01
			cpg_x[2]= 0.01
			cpg_y[2]= 0.01
			cpg_x[3]= -0.01
			cpg_y[3]= -0.01
			cpg_x[4]= -0.01
			cpg_y[4]= -0.01
			cpg_x[5]= 0.01
			cpg_y[5]= 0.01
			cpg_x_next = cpg_x
			cpg_y_next = cpg_y
			cx0_offset = np.array([np.pi/4, np.pi/4, 0, 0, -np.pi/4, -np.pi/4])
			#cx0_offset = np.zeros(6)
			cy0_offset = np.array([np.pi/16, np.pi/16, np.pi/16, np.pi/16, np.pi/16, np.pi/16])
			#cy0_offset = np.zeros(6)
			while not rospy.is_shutdown():    
				#CPG
				for i in range(6):  #for each leg
					r = math.sqrt((cpg_x[i]-cx0_offset[i])**2+(cpg_y[i]-cy0_offset[i])**2)
					ksum0[i] = 0
					for row in range(np.shape(K_array)[0]):
						for col in range(np.shape(K_array)[1]):
							if row == i:
								#print(K_array[r][c])
								ksum0[i] = (ksum0[i] + K_array[row][col])*cpg_y[i]
								#print(ksum)
					cpg_x_next[i] = (alpha*(mew - (r*r))*cpg_x[i] - omega*(cpg_y[i]))*dt + cpg_x[i]
					cpg_y_next[i] = (beta*(mew - (r*r))*cpg_y[i] + omega*(cpg_x[i])+ lambda_cs*ksum0[i])*dt + cpg_y[i] 
				
				if not self.initialized:
					self.rate.sleep()
					continue

				# Create Joint State
				joint_state = JointState()
				joint_state.header.stamp = rospy.Time.now()
				
				# for i in range(self.num_joints):
							#     joint_state.name.append('bendy_input_M{}_S{}'.format(i+1,i+1))
				joint_state.name = self.joint_names

				# Assume we are using positional control
				joint_state.velocity = [float(30)]*self.num_joints
				joint_state.effort = [float(30)]*self.num_joints
				joint_state.position  = np.copy(self.initial_joint_positions)

				# Set joint positions

				#0123456789 10 11 12 13 14 15 16 17 index
				#0000001111  1  1  2  2  2  2  2  2 joint number (joint_i var)
				#proximal interm        distal      joint name
				#1234561234  5  6  1  2  3  4  5  6 leg number   (leg_i var)

				# Translation:  
				# CPG 0 = leg 1 = sim leg 3 
				# CPG 1 = leg 2 = sim leg 0 
				# CPG 2 = leg 3 = sim leg 4 
				# CPG 3 = leg 4 = sim leg 1 
				# CPG 4 = leg 5 = sim leg 5 
				# CPG 5 = leg 6 = sim leg 2

				for i in range(self.num_joints):
					joint_i = i//6
					simleg_i = i%6
					if simleg_i == 0:
						CPG_index = 1
						if joint_i == 0: #proximal joint
							joint_state.position[i] = (8 * cpg_x_next[CPG_index]) + self.initial_joint_positions[i]
						if joint_i == 1: #intermediate joint
							joint_state.position[i] = (20 * cpg_y_next[CPG_index]) + self.initial_joint_positions[i]
					if simleg_i == 1:
						CPG_index = 3
						if joint_i == 0: #proximal joint
							joint_state.position[i] = (8 * cpg_x_next[CPG_index]) + self.initial_joint_positions[i]
						if joint_i == 1: #intermediate joint
							joint_state.position[i] = (20 * cpg_y_next[CPG_index]) + self.initial_joint_positions[i]
					if simleg_i == 2:
						CPG_index = 5
						if joint_i == 0: #proximal joint
							joint_state.position[i] = (8 * cpg_x_next[CPG_index]) + self.initial_joint_positions[i]
						if joint_i == 1: #intermediate joint
							joint_state.position[i] = (20 * cpg_y_next[CPG_index]) + self.initial_joint_positions[i]
					if simleg_i == 3:
						CPG_index = 0
						if joint_i == 0: #proximal joint
							joint_state.position[i] = (8 * cpg_x_next[CPG_index]) + self.initial_joint_positions[i]
						if joint_i == 1: #intermediate joint
							joint_state.position[i] = (20 * cpg_y_next[CPG_index]) + self.initial_joint_positions[i]
					if simleg_i == 4:
						CPG_index = 2
						if joint_i == 0: #proximal joint
							joint_state.position[i] = (8 * cpg_x_next[CPG_index]) + self.initial_joint_positions[i]
						if joint_i == 1: #intermediate joint
							joint_state.position[i] = (20 * cpg_y_next[CPG_index]) + self.initial_joint_positions[i]
					if simleg_i == 5:
						CPG_index = 4
						if joint_i == 0: #proximal joint
							joint_state.position[i] = (8 * cpg_x_next[CPG_index]) + self.initial_joint_positions[i]
						if joint_i == 1: #intermediate joint
							joint_state.position[i] = (20 * cpg_y_next[CPG_index]) + self.initial_joint_positions[i]
					
						# print("CPG_X: " + str(cpg_x_next[leg_i]))
						# print("CPG_Y: " + str(cpg_y_next[leg_i]))
						# print("Joint " + str(joint_i) + ": " + str(joint_state.position[i]))
				self.joint_cmd_pub.publish(joint_state)
					#else:
					#    joint_state.position[i] = amplitudes[joint_i,leg_i]*np.sin(t + phase_offsets[joint_i,leg_i]) # + const_offsets[joint_i, leg_i]
					#    if joint_i >= 1:
					#        joint_state.position[i] = max(0, joint_state.position[i])
					#    joint_state.position[i] += self.initial_joint_positions[i]
					# self.joint_cmd_pub.publish(joint_state)
				t += dt
				cpg_x = cpg_x_next
				cpg_y = cpg_y_next
				self.rate.sleep()
		else:
			print("Complex Model")
			omega = 40/(np.pi*2) #oscillation frequency in rad/s
			gamma = 40 #from paper
			gait_a = np.pi/18 #from paper
			gait_b = np.pi/6 #from paper
			gait_n = 4 #from paper
			mew = 1 #from paper
			#cx0_offset = np.zeros(6)
			#cy0_offset = np.zeros(6)
			cx0_offset = np.array([np.pi/4, np.pi/4, 0, 0, -np.pi/4, -np.pi/4]) 
			#cx0_offset = np.array([np.pi/16, np.pi/16, 0, 0, -np.pi/16, -np.pi/16]) 
			cy0_offset = np.array([np.pi/16, np.pi/16, np.pi/16, np.pi/16, np.pi/16, np.pi/16]) #from paper
			t = 0
			dt = 0.0001
			cpg_x = np.zeros(6)
			cpg_y = np.zeros(6)
			cpg_x_next = np.zeros(6)
			cpg_y_next = np.zeros(6)
			ksum0 = np.zeros(6)
			lambda_cs = 0.25 #K array coupling strength
			#coupling matrix K for tripod gait on hexapod
			K_array =   np.array(
						[[0, -1, -1, 1, 1, -1],
						[-1, 0, 1, -1, -1, 1],
						[-1, 1, 0, -1, -1, 1],
						[1, -1, -1, 0, 1, -1],
						[1, -1, -1, 1, 0, -1],
						[-1, 1, 1, -1, -1, 0]])
			while not rospy.is_shutdown():    
				#CPG
				for i in range(6):
					H = np.absolute(((cpg_x[i]-cx0_offset[i])/gait_a)**gait_n) + np.absolute(((cpg_y[i]-cy0_offset[i])/gait_b)**gait_n)
					dHdx = (gait_n/gait_a)*((cpg_x[i]-cx0_offset[i])/gait_a)**(gait_n-1)
					dHdy = (gait_n/gait_b)*((cpg_y[i]-cy0_offset[i])/gait_b)**(gait_n-1)
					
					#reset ksum0
					ksum0[i] = 0
					for r in range(np.shape(K_array)[0]):
						for c in range(np.shape(K_array)[1]):
							if r == i:
								#print(K_array[r][c])
								ksum0[i] = ksum0[i] + (K_array[r][c]*(cpg_y[i]-cy0_offset[i]))
								#print(ksum)
					cpg_x_next[i] = (gamma*(mew - H)*dHdx - omega*(dHdy))*dt + cpg_x[i]
					cpg_y_next[i] = (gamma*(mew - H)*dHdy + omega*(dHdx) + lambda_cs*ksum0[i])*dt + cpg_y[i]
				if not self.initialized:
					self.rate.sleep()
					continue

				# Create Joint State
				joint_state = JointState()
				joint_state.header.stamp = rospy.Time.now()
				
				# for i in range(self.num_joints):
							#     joint_state.name.append('bendy_input_M{}_S{}'.format(i+1,i+1))
				joint_state.name = self.joint_names

				# Assume we are using positional control
				joint_state.velocity = [float(5)]*self.num_joints
				joint_state.effort = [float(5)]*self.num_joints
				joint_state.position  = np.copy(self.initial_joint_positions)

				# Set joint positions
				for i in range(self.num_joints):
					joint_i = i//6
					simleg_i = i%6
					if simleg_i == 0:
						CPG_index = 1
						if joint_i == 0: #proximal joint
							joint_state.position[i] = (10 * cpg_x_next[CPG_index]) + self.initial_joint_positions[i]
						if joint_i == 1: #intermediate joint
							joint_state.position[i] = (30 * cpg_y_next[CPG_index]) + self.initial_joint_positions[i]
					if simleg_i == 1:
						CPG_index = 3
						if joint_i == 0: #proximal joint
							joint_state.position[i] = (10 * cpg_x_next[CPG_index]) + self.initial_joint_positions[i]
						if joint_i == 1: #intermediate joint
							joint_state.position[i] = (30 * cpg_y_next[CPG_index]) + self.initial_joint_positions[i]
					if simleg_i == 2:
						CPG_index = 5
						if joint_i == 0: #proximal joint
							joint_state.position[i] = (10 * cpg_x_next[CPG_index]) + self.initial_joint_positions[i]
						if joint_i == 1: #intermediate joint
							joint_state.position[i] = (30 * cpg_y_next[CPG_index]) + self.initial_joint_positions[i]
					if simleg_i == 3:
						CPG_index = 0
						if joint_i == 0: #proximal joint
							joint_state.position[i] = (10 * cpg_x_next[CPG_index]) + self.initial_joint_positions[i]
						if joint_i == 1: #intermediate joint
							joint_state.position[i] = (30 * cpg_y_next[CPG_index]) + self.initial_joint_positions[i]
					if simleg_i == 4:
						CPG_index = 2
						if joint_i == 0: #proximal joint
							joint_state.position[i] = (10 * cpg_x_next[CPG_index]) + self.initial_joint_positions[i]
						if joint_i == 1: #intermediate joint
							joint_state.position[i] = (30 * cpg_y_next[CPG_index]) + self.initial_joint_positions[i]
					if simleg_i == 5:
						CPG_index = 4
						if joint_i == 0: #proximal joint
							joint_state.position[i] = (10 * cpg_x_next[CPG_index]) + self.initial_joint_positions[i]
						if joint_i == 1: #intermediate joint
							joint_state.position[i] = (30 * cpg_y_next[CPG_index]) + self.initial_joint_positions[i]
				self.joint_cmd_pub.publish(joint_state)
					#else:
					#    joint_state.position[i] = amplitudes[joint_i,leg_i]*np.sin(t + phase_offsets[joint_i,leg_i]) # + const_offsets[joint_i, leg_i]
					#    if joint_i >= 1:
					#        joint_state.position[i] = max(0, joint_state.position[i])
					#    joint_state.position[i] += self.initial_joint_positions[i]
					# self.joint_cmd_pub.publish(joint_state)
				t += dt
				cpg_x = cpg_x_next
				cpg_y = cpg_y_next
				self.rate.sleep()
		rospy.spin()
       


	def joint_fb_callback(self, msg):
		if not self.initialized:
			# Initialize current joint positions from feedback
			self.initial_joint_positions = np.array(msg.position)
			self.initialized = True
		self.joint_names = msg.name
		self.num_joints = len(msg.name)
		
  		# update imu dictionary
		for i in range in self.num_joints:
			self.imu[i] = dict()
			self.imu[i]['position'] = msg.position[i]
			self.imu[i]['velocity'] = 0
			self.imu[i]['effort'] = 0
			if i < len(msg.velocity):
				self.imu[i]['velocity'] = msg.velocity[i]
			if i < len(msg.effort):
				self.imu[i]['effort'] = msg.effort[i]
		print(self.imu)
  


if __name__ == '__main__':
	rospy.init_node('eigenbot_joint_pub')
	eigenbot_joint_pub = EigenbotJointPub()
	eigenbot_joint_pub.main_loop() 
