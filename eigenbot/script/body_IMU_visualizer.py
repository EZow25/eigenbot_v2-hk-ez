#!/usr/bin/env python3


#Based on eigenbot_joint_pub.py and eigenbot_joint_controller.py
#Last updated 5/23/23 by hkou@andrew.cmu.edu
# import numpy as np
# from vpython import *
# import rospy
# import math
# import rosnode
# from sensor_msgs.msg import JointState
# from geometry_msgs.msg import PoseStamped


import numpy as np
from vpython import *
import rospy
import math
import rosnode
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

scene.range=6
scene.background=color.yellow
scene.forward=vector(-1,-1,-1)
scene.width=1200
scene.height=1080
xarrow=arrow(length=2, shaftwidth=.1, color=color.red,axis=vector(1,0,0))
yarrow=arrow(length=2, shaftwidth=.1, color=color.green,axis=vector(0,1,0))
zarrow=arrow(length=4, shaftwidth=.1, color=color.blue,axis=vector(0,0,1))
 
frontArrow=arrow(length=2,shaftwidth=.1,color=color.purple,axis=vector(1,0,0))
upArrow=arrow(length=2.5,shaftwidth=.1,color=color.black,axis=vector(0,1,0))
sideArrow=arrow(length=2.5,shaftwidth=.1,color=color.orange,axis=vector(0,0,1))
bBoard=box(length=1,width=3.5,height=2,opacity=.8,pos=vector(0,0,0,))
myObj=compound([bBoard])


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
        rate(200)
        k=vector(cos(yaw)*cos(pitch), sin(pitch),sin(yaw)*cos(pitch))
        y=vector(0,1,0)
        s=cross(k,y)
        v=cross(s,k)
        vrot=v*cos(roll)+cross(k,v)*sin(roll) #rodriguez formula for roll
 
        frontArrow.axis=k
        sideArrow.axis=cross(k,vrot)
        upArrow.axis=vrot
        myObj.axis=k
        myObj.up=vrot
        sideArrow.length=2
        frontArrow.length=-2
        upArrow.length=-2

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
        

    def main_loop(self):
        # parameters for alternating tripod
        # amplitudes = np.tile(np.array([np.pi/8, 3*np.pi/16, np.pi/4])[:,None], (1,6))
        amplitudes = np.tile(np.array([np.pi/8, 2*np.pi/16, np.pi/8])[:,None], (1,6))
        # const_offsets = np.array([[0,0,0,0,0,0], 
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
        # for test without 90deg module in hexapod
        # const_offsets[1,:] -= np.pi/4
        # const_offsets[2,:] += np.pi/4
        t = 0
        dt = np.pi/10
        while not rospy.is_shutdown():
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
            joint_state.velocity = [float('3')]*self.num_joints
            joint_state.effort = [float('3')]*self.num_joints
            joint_state.position  = np.copy(self.initial_joint_positions)

            # Set joint positions
            for i in range(self.num_joints):
                joint_i = i//6
                leg_i = i%6
                joint_state.position[i] = amplitudes[joint_i,leg_i]*np.sin(t + phase_offsets[joint_i,leg_i]) # + const_offsets[joint_i, leg_i]
                if joint_i >= 1:
                    joint_state.position[i] = max(0, joint_state.position[i])
                joint_state.position[i] += self.initial_joint_positions[i]
            self.joint_cmd_pub.publish(joint_state)

            #visualize body orientation
            # rate(200)
            
            # self.k=vector(cos(self.body_orientation.yaw)*cos(self.body_orientation.pitch), sin(self.body_orientation.pitch),sin(self.body_orientation.yaw)*cos(self.body_orientation.pitch))
            # self.y=vector(0,1,0)
            # self.s=cross(self.k,self.y)
            # self.v=cross(self.s,self.k)
            # self.vrot=self.v*cos(self.body_orientation.roll)+cross(self.k,self.v)*sin(self.body_orientation.roll)
    
            # frontArrow.axis=self.k
            # sideArrow.axis=cross(self.k,self.vrot)
            # upArrow.axis=self.vrot
            # myObj.axis=self.k
            # myObj.up=self.vrot
            # sideArrow.length=2
            # frontArrow.length=4
            # upArrow.length=1
            

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