#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist

# Initial pose of the box
box_pose = Pose()

# Control scaling factor (adjust this to tune movement sensitivity)
SCALE = 0.01

def joy_callback(msg):
    global box_pose

    # Assuming msg.axes[0], [1], [2] are x, y, z of the Falcon
    dx = msg.axes[0] * SCALE
    dy = msg.axes[1] * SCALE
    dz = msg.axes[2] * SCALE

    # Update box position
    box_pose.position.x += dx
    box_pose.position.y += dy
    box_pose.position.z += dz

    # Create and publish new model state
    state_msg = ModelState()
    state_msg.model_name = 'floating_box'
    state_msg.pose = box_pose
    state_msg.twist = Twist()
    state_msg.reference_frame = 'world'

    pub.publish(state_msg)

if __name__ == '__main__':
    rospy.init_node('falcon_box_control')

    # Publisher to move the box
    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)

    # Set initial pose (can be adjusted)
    box_pose.position.x = 1.4
    box_pose.position.y = 0
    box_pose.position.z = 2.5

    # Subscribe to left Falcon (change to joystick1 if you prefer right Falcon)
    rospy.Subscriber('/falcon/joystick', Joy, joy_callback)

    rospy.loginfo("Falcon box control node started.")
    rospy.spin()

