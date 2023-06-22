#!/usr/bin/env python3


#Based on eigenbot_joint_pub.py and eigenbot_joint_controller.py
#Last updated 6/15/23 by hkou@andrew.cmu.edu


import numpy as np
import rospy
import math
import rosnode
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped


def wrap_to_pi(angle):
    return np.remainder(angle + np.pi,  np.pi*2) - np.pi

def quaternion2RPY(quat_dict, RPY_dict):
        q0=quat_dict.w
        q1=quat_dict.x
        q2=quat_dict.y
        q3=quat_dict.z

        roll=-math.atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2))
        pitch=2*math.atan2(math.sqrt(1+2*(q0*q2-q1*q3)),math.sqrt(1-2*(q0*q2-q1*q3))) - np.pi/2
        #pitch=math.asin(2*(q0*q2-q3*q1))
        yaw=-math.atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3))
        RPY_dict.update({'roll':roll, 'pitch':pitch, 'yaw': yaw})
        #print("roll: ", roll, "pitch: ", pitch, "yaw: ", yaw)
        
def compute_distal_angle(proximal_angle, intermediate_angle):
    distal_angle = 0
    return distal_angle

class EigenbotJointPub():
    def __init__(self):
        self.initialized = False
        self.num_joints = 18
        self.joint_positions = [float('nan')]*self.num_joints
        self.joint_names = []
        self.initial_joint_positions = np.zeros(self.num_joints)
        self.rate = rospy.Rate(10)
        self.body_quaternion = {} #dictionary variable for body orientation, global? or public...
        self.body_position = {}   #dictionary variable for body position
        self.body_orientation = {} #RPY
        self.joint_cmd_pub = rospy.Publisher('/eigenbot/joint_cmd', JointState, queue_size=1)
        self.joint_fb_sub = rospy.Subscriber('/eigenbot/joint_fb', JointState, self.joint_fb_callback)
        self.body_IMU_fb_sub = rospy.Subscriber('/eigenbot/body_IMU_fb', PoseStamped, self.body_IMU_fb_callback)
        
        #Posture control variables
        self.cpg_x = 0
        self.cpg_x_next = 0
        self.cpg_y = 0
        self.cpg_y_next = 0
        self.forw_velocity_omega = 10
        self.gamma = 40 #from paper
        self.gait_a = np.pi/18 #from paper
        self.gait_b = np.pi/6 #from paper
        self.gait_n = 4 #from paper
        self.mew = 1 #from paper
        self.cx0 = np.array([0,0,0,0,0,0])
        self.cy0 = np.array([np.pi/16,np.pi/16, np.pi/16, np.pi/16, np.pi/16, np.pi/16]) #from paper
        self.R_target_pose = np.array([0,0,0]) #quaternion? RPY?
        self.T_goal_pose = np.array([0,0,0])
        self.P_current_pose = np.array([0,0,0])
        self.theta_z = 0 #heading??
        self.Rz_transform = np.array([[math.cos(self.theta_z), -math.sin(self.theta_z), 0],
                                      [math.sin(self.theta_z), math.cos(self.theta_z), 0],
                                      [0,0,1]])


    def main_loop(self):
        # parameters for alternating tripod
        # amplitudes = np.tile(np.array([np.pi/8, 3*np.pi/16, np.pi/4])[:,None], (1,6))
        amplitudes = np.tile(np.array([np.pi/8, 2*np.pi/16, np.pi/8])[:,None], (1,6))
        #     [1,1,1,1,1,1], [0,0,0,0,0,0]])*amplitudes
        const_offsets = np.array([
            [-1,1,-1,1,-1,1],
            [0,0,0,0,0,0],
            [0,0,0,0,0,0]
        ])*amplitudes
        phase_offsets = np.array([
            [np.pi/2,-np.pi/2,np.pi/2,-np.pi/2,np.pi/2,-np.pi/2],
            [0,np.pi,0,np.pi,0,np.pi],
            [0,np.pi,0,np.pi,0,np.pi]
        ])

        #Timing constants
        t = 0
        dt = np.pi/10

        #set target constant orientation T
        #set tripod gait coupling matrix K
        #Setup initial offsets based on specific leg 

        while not rospy.is_shutdown():
            if not self.initialized:
                self.rate.sleep()
                continue

            
            #Compute CPG joint angles:
            #Find dx/dt, dy/dt for this time step using Euler forward model
            self.cpg_x_next = -self.forw_velocity_omega*self.cpg_y+self.gamma*(self.mew*self.mew-math.sqrt(self.cpg_x*self.cpg_x+self.cpg_y*self.cpg_y))*self.cpg_x
            self.cpg_y_next = self.forw_velocity_omega*self.cpg_x+self.gamma*(self.mew*self.mew-math.sqrt(self.cpg_x*self.cpg_x+self.cpg_y*self.cpg_y))*self.cpg_y

            #Set Joint States
            





            # Create Joint State (original tripod sin gait)
            joint_state = JointState()
            joint_state.header.stamp = rospy.Time.now()
            
            # for i in range(self.num_joints):
            #     joint_state.name.append('bendy_input_M{}_S{}'.format(i+1,i+1))
            joint_state.name = self.joint_names

            # Assume we are using positional control
            joint_state.velocity = [float('3')]*self.num_joints
            joint_state.effort = [float('3')]*self.num_joints
            joint_state.position  = np.copy(self.initial_joint_positions)

            # Set joint positions
            #0123456789 10 11 12 13 14 15 16 17 index
            #0000001111  1  1  2  2  2  2  2  2 joint number (joint_i var)
            #proximal interm        distal      joint name
            #1234561234  5  6  1  2  3  4  5  6 leg number   (leg_i var)

            #each position variable contains 3 rows (joints) x 6 cols (legs). 
            #iterate through proximal joint on every leg, then to the intermediate joint on every leg, and finally all the distals.
            
            for i in range(self.num_joints):
                joint_i = i//6 #floor division 
                
                leg_i = i%6 #modulus
                if joint_i == 0: #proximal
                    joint_state.position[i] = self.cpg_x_next
                elif joint_i == 1:
                    joint_state.position[i] = self.cpg_y_next#max(self.cpg_y, self.cy0[i])
                elif joint_i == 2:
                    proximal_angle = 0
                    intermediate_angle = 0
                    joint_state.position[i] = compute_distal_angle(proximal_angle, intermediate_angle)
                #joint_state.position[i] = amplitudes[joint_i,leg_i]*np.sin(t + phase_offsets[joint_i,leg_i]) # + const_offsets[joint_i, leg_i]
                
                
                
                if joint_i >= 1: #intermediate and distal joints
                    joint_state.position[i] = max(0, joint_state.position[i])
                joint_state.position[i] += self.initial_joint_positions[i]
                
                #print("joint position" + str(i) + ": " + str(joint_state.position[i]))
                print(joint_state)
            self.joint_cmd_pub.publish(joint_state)
            
            
            self.cpg_x = self.cpg_x_next
            self.cpg_y = self.cpg_y_next
            t += dt
            self.rate.sleep()
    
    


    def joint_fb_callback(self, msg):
        if not self.initialized:
            # Initialize current joint positions from feedback
            self.initial_joint_positions = np.array(msg.position)
            self.initialized = True
        self.joint_names = msg.name
        self.num_joints = len(msg.name)

    def body_IMU_fb_callback(self, msg):
        self.body_position = msg.pose.position
        self.body_quaternion = msg.pose.orientation
        quaternion2RPY(self.body_quaternion, self.body_orientation)

        # if not self.initialized:
        #     # Initialize current joint positions from feedback
        #     self.initial_joint_positions = np.array(msg.position)
        #     self.initialized = True
        # self.joint_names = msg.name
        # self.num_joints = len(msg.name)

    

        


if __name__ == '__main__':
    rospy.init_node('eigenbot_joint_pub')
    eigenbot_joint_pub = EigenbotJointPub()
    eigenbot_joint_pub.main_loop()    