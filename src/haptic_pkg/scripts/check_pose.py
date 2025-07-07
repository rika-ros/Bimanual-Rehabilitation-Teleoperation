#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates

def model_states_callback(msg):
    # Find the index of the box in the list of model names
    index = msg.name.index("falcon_cursor")

    # Get the pose of the box from the list of poses
    pose = msg.pose[index]

    # Print the position of the box
    print("Position: (%.2f, %.2f, %.2f)" % (pose.position.x, pose.position.y, pose.position.z))

def get_box_position():
    rospy.init_node('get_box_position')
    rospy.Subscriber("/gazebo/model_states", ModelStates, model_states_callback)
    rospy.spin()

if __name__ == '__main__':
    get_box_position()
