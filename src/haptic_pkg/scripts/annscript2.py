#!/usr/bin/env python3

import rospy
import time
import numpy as np
from geometry_msgs.msg import Wrench, WrenchStamped
from sensor_msgs.msg import Joy
from ros_falcon.msg import falconForces
from gazebo_msgs.msg import LinkStates

# Constants
M = 1.02
k = 5      # stiffness gain
d = 1      # damping gain
sensitivity = np.array([20, 20, 50])  # Sensitivity per axis (x, y, z)

# State variables
pos_falcon_left = np.zeros(3)
pos_falcon_right = np.zeros(3)
pos_box = np.zeros(3)
vel_box = np.zeros(3)
force_box_left = np.zeros(3)
force_box_right = np.zeros(3)

# Publishers
pub_box_force_left = rospy.Publisher("/force_left", Wrench, queue_size=1)
pub_box_force_right = rospy.Publisher("/force_right", Wrench, queue_size=1)
pub_falcon_force_left = rospy.Publisher("/falconForce", falconForces, queue_size=1)
pub_falcon_force_right = rospy.Publisher("/falconForce1", falconForces, queue_size=1)

# Apply deadzone
def apply_deadzone(val, threshold=0.05):
    return val if abs(val) > threshold else 0.0

# Callbacks
def callback_pos_falcon_left(data):
    global pos_falcon_left
    pos_falcon_left = np.array([data.axes[0], data.axes[1], data.axes[2]])

def callback_pos_falcon_right(data):
    global pos_falcon_right
    pos_falcon_right = np.array([data.axes[0], data.axes[1], data.axes[2]])

def callback_force_box_left(data):
    global force_box_left
    force_box_left = np.array([data.wrench.force.x, data.wrench.force.y, data.wrench.force.z])

def callback_force_box_right(data):
    global force_box_right
    force_box_right = np.array([data.wrench.force.x, data.wrench.force.y, data.wrench.force.z])

def box_pos_cb(data):
    global pos_box, vel_box
    try:
        index = data.name.index("floating_box::base_link")
        pos_box = np.array([
            data.pose[index].position.x,
            data.pose[index].position.y,
            data.pose[index].position.z
        ])
        vel_box = np.array([
            data.twist[index].linear.x,
            data.twist[index].linear.y,
            data.twist[index].linear.z
        ])
    except ValueError:
        rospy.logwarn_throttle(5, "Box model not found in /gazebo/link_states")

# Force publishers
def publish_wrench(pub, force):
    msg = Wrench()
    msg.force.x = force[0]
    msg.force.y = force[1]
    msg.force.z = force[2]
    pub.publish(msg)

def publish_falcon_force(pub, force):
    msg = falconForces()
    msg.X = force[0]
    msg.Y = force[1]
    msg.Z = force[2]
    pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node("aan_3d_controller", anonymous=True)

    rospy.Subscriber("/falcon/joystick", Joy, callback_pos_falcon_left, queue_size=1)
    rospy.Subscriber("/falcon/joystick1", Joy, callback_pos_falcon_right, queue_size=1)
    rospy.Subscriber("/left/force_feedback", WrenchStamped, callback_force_box_left, queue_size=1)
    rospy.Subscriber("/right/force_feedback", WrenchStamped, callback_force_box_right, queue_size=1)
    rospy.Subscriber("/gazebo/link_states", LinkStates, box_pos_cb, queue_size=1)

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        # Compute control error
        err_left = pos_box - pos_falcon_left * sensitivity
        err_right = pos_box - pos_falcon_right * sensitivity

        vel_err = vel_box  # Assuming desired velocity = 0

        # Assistive control using spring-damper model (AAN)
        u_left_box = -k * err_left - d * vel_err
        u_right_box = -k * err_right - d * vel_err

        # Haptic feedback to Falcon (equal and opposite)
        u_left_falcon = -u_left_box
        u_right_falcon = -u_right_box

        # Publish assistive forces to box
        publish_wrench(pub_box_force_left, u_left_box)
        publish_wrench(pub_box_force_right, u_right_box)

        # Publish haptic forces to Falcon
        publish_falcon_force(pub_falcon_force_left, u_left_falcon)
        publish_falcon_force(pub_falcon_force_right, u_right_falcon)

        rate.sleep()

