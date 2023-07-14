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
			dt = 0.001
			cpg_x = [0.01] * 6
			cpg_y = [0.01] * 6
			while not rospy.is_shutdown():    
				#CPG
				for i in range(6):  
					r = math.sqrt(cpg_x[i]**2+cpg_y[i]**2)
					ksum0[i] = 0
					for row in range(np.shape(K_array)[0]):
						for col in range(np.shape(K_array)[1]):
							if row == i:
								#print(K_array[r][c])
								ksum0[i] = (ksum0[i] + K_array[row][col])*cpg_y[i]
								#print(ksum)
					cpg_x[i] = (alpha*(mew - r)*cpg_x[i] - omega*(cpg_y[i]))*dt + cpg_x[i]
					cpg_y[i] = (beta*(mew - r)*cpg_y[i] + omega*(cpg_x[i])+ lambda_cs*ksum0[i])*dt + cpg_y[i] 
				
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
				for i in range(self.num_joints):
					joint_i = i//6
					leg_i = i%6
					if leg_i == 2:
						if joint_i == 0:
							joint_state.position[i] = (10 * cpg_x[leg_i]) + self.initial_joint_positions[i]
						if joint_i == 1:
							joint_state.position[i] = (30 * cpg_y[leg_i]) + self.initial_joint_positions[i]
						print("CPG_X: " + str(cpg_x[leg_i]))
						print("CPG_Y: " + str(cpg_y[leg_i]))
						print("Joint " + str(joint_i) + ": " + str(joint_state.position[i]))
				self.joint_cmd_pub.publish(joint_state)
					#else:
					#    joint_state.position[i] = amplitudes[joint_i,leg_i]*np.sin(t + phase_offsets[joint_i,leg_i]) # + const_offsets[joint_i, leg_i]
					#    if joint_i >= 1:
					#        joint_state.position[i] = max(0, joint_state.position[i])
					#    joint_state.position[i] += self.initial_joint_positions[i]
					# self.joint_cmd_pub.publish(joint_state)
				t += dt
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
			#cx0_offset = np.array([np.pi/4, np.pi/4, 0, 0, -np.pi/4, -np.pi/4]) 
			cx0_offset = np.array([np.pi/16, np.pi/16, 0, 0, -np.pi/16, -np.pi/16]) 
			cy0_offset = np.array([np.pi/16, np.pi/16, np.pi/16, np.pi/16, np.pi/16, np.pi/16]) #from paper
			t = 0
			dt = 0.0001
			cpg_x = np.zeros(6)
			cpg_y = np.zeros(6)
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
					cpg_x[i] = (gamma*(mew - H)*dHdx - omega*(dHdy))*dt + cpg_x[i]
					cpg_y[i] = (gamma*(mew - H)*dHdy + omega*(dHdx) + lambda_cs*ksum0[i])*dt + cpg_y[i]
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
					leg_i = i%6
					if leg_i == 2:
						if joint_i == 0:
							joint_state.position[i] = cpg_x[leg_i] + self.initial_joint_positions[i]
						if joint_i == 1:
							joint_state.position[i] = cpg_y[leg_i] + self.initial_joint_positions[i]
						print("CPG_X: " + str(cpg_x[leg_i]))
						print("CPG_Y: " + str(cpg_y[leg_i]))
						print("Joint " + str(joint_i) + ": " + str(joint_state.position[i]))
				self.joint_cmd_pub.publish(joint_state)
					#else:
					#    joint_state.position[i] = amplitudes[joint_i,leg_i]*np.sin(t + phase_offsets[joint_i,leg_i]) # + const_offsets[joint_i, leg_i]
					#    if joint_i >= 1:
					#        joint_state.position[i] = max(0, joint_state.position[i])
					#    joint_state.position[i] += self.initial_joint_positions[i]
					# self.joint_cmd_pub.publish(joint_state)
				t += dt
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
		print(self.joint_names)
  


if __name__ == '__main__':
	rospy.init_node('eigenbot_joint_pub')
	rospy.init_node('eigenbot_joint_sub', anonymous=True)
	eigenbot_joint_pub = EigenbotJointPub()
	eigenbot_joint_pub.main_loop() 
