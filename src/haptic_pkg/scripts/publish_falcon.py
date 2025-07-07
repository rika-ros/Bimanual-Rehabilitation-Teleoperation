#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import WrenchStamped
from ros_falcon.msg import falconForces

def left_force_callback(msg):
    # Create a falconForces message
    force_msg = falconForces()

    # Set the force values in the message
    force_msg.X =  msg.wrench.force.x
    force_msg.Y =  msg.wrench.force.y
    force_msg.Z =  msg.wrench.force.z

    # Publish the message
    pub = rospy.Publisher('falconForce', falconForces, queue_size=5)
    pub.publish(force_msg)

def right_force_callback(msg):
    # Create a falconForces message
    force_msg = falconForces()

    # Set the force values in the message
    force_msg.X =  msg.wrench.force.x
    force_msg.Y =  msg.wrench.force.y
    force_msg.Z =  msg.wrench.force.z

    # Publish the message
    pub = rospy.Publisher('falconForce1', falconForces, queue_size=5)
    pub.publish(force_msg)

def subscribe_and_publish():
    rospy.init_node('subscribe_and_publish')
    rospy.Subscriber("/left/force_feedback", WrenchStamped, left_force_callback)
    rospy.Subscriber("/right/force_feedback", WrenchStamped, right_force_callback)
    rospy.spin()

if __name__ == '__main__':
    subscribe_and_publish()