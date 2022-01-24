#!/usr/bin/env python

"""
@author: jwhitman@cmu.edu
Julian Whitman
This ros node listens for eigenbot/topology messages, then takes the first one,
then decodes it, prints out the assembly info, and then assembles and launches it.
note: make sure to run chmod +x description_listener.py first to make it executable
"""
# see http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29

import rospy
# import message_parsing
# import description_assembler
from topology_message_tools import parse_topology
from eigenbot_joystick_ros import joint_state_talker
from std_msgs.msg import String
last_data = []

def callback(data):
    global last_data
    if not(last_data==data.data): # only log new data. If it's the same, ignore it.
        # rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.data)
     	    if len(data.data)>0:


        	    # module_ID, module_attachments, module_ID_serials = message_parsing.parse_message_str(data.data)
        	    # print module_ID
        	    # print module_ID_serials
        	    # print module_attachments
                module_types_str, graph_edges, module_ids = parse_topology(data.data)
                print('------ parsed to:')
                print('module_types_str = ')
                print(module_types_str)
                print('graph_edges = ')
                print(graph_edges)
                print('module_ids = ')
                print(module_ids)
                print('---------')

                joint_state_talker(module_types_str, graph_edges, module_ids)

                # to end the node spin, use
                # rospy.signal_shutdown("Parsed message, shutting this node down")



def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    # rospy.init_node('description_listener', anonymous=True)
    rospy.init_node('description_listener')
    rate = rospy.Rate(50) # 10hz

    # if we want to run the callback only once, use this
    data = rospy.wait_for_message("/eigenbot/topology", String)
    module_types_str, graph_edges, module_ids = parse_topology(data.data)
    joint_state_talker(module_types_str, graph_edges, module_ids)



    # if we want to run the callback for every new message, use this
    # rospy.Subscriber("eigenbot/topology", String, callback)
    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()

if __name__ == '__main__':
    listener()
