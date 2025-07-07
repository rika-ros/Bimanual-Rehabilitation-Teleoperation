#!/usr/bin/env python3

import rospy, time
from geometry_msgs.msg import Wrench, WrenchStamped
from sensor_msgs.msg import Joy
from ros_falcon.msg import falconForces
from gazebo_msgs.msg import LinkStates
import numpy as np

rospy.init_node("Box_control", anonymous=True)

# Publishers
force_Box = rospy.Publisher("/force_B_fromsides", Wrench, queue_size=1)
force_to_left_box = rospy.Publisher("/force_left", Wrench, queue_size=1)
force_to_right_box = rospy.Publisher("/force_right", Wrench, queue_size=1)
force_to_left_falcon = rospy.Publisher("/falconForce1", falconForces, queue_size=1)
force_to_right_falcon = rospy.Publisher("/falconForce", falconForces, queue_size=1)

# Messages
msg_force_to_left_box = Wrench()
msg_force_to_right_box = Wrench()
msg_force_to_left_falcon = falconForces()
msg_force_to_right_falcon = falconForces()

# State Variables
pos_falcon_left = np.zeros(3)
pos_falcon_right = np.zeros(3)
pos_box = np.zeros(3)
vel_box = np.zeros(3)
force_fb_left_box = np.zeros(3)
force_fb_right_box = np.zeros(3)

def left_box(x, y, z):
    msg_force_to_left_box.force.x = x
    msg_force_to_left_box.force.y = y
    msg_force_to_left_box.force.z = z
    force_to_left_box.publish(msg_force_to_left_box)

def right_box(x, y, z):
    msg_force_to_right_box.force.x = x
    msg_force_to_right_box.force.y = y
    msg_force_to_right_box.force.z = z
    force_to_right_box.publish(msg_force_to_right_box)

def left_falcon(x, y, z):
    msg_force_to_left_falcon.X = x
    msg_force_to_left_falcon.Y = y
    msg_force_to_left_falcon.Z = z
    force_to_left_falcon.publish(msg_force_to_left_falcon)

def right_falcon(x, y, z):
    msg_force_to_right_falcon.X = x
    msg_force_to_right_falcon.Y = y
    msg_force_to_right_falcon.Z = z
    force_to_right_falcon.publish(msg_force_to_right_falcon)

def callback_left_falcon(data):
    global pos_falcon_left
    pos_falcon_left = np.array([data.axes[0], data.axes[1], data.axes[2]])

def callback_right_falcon(data):
    global pos_falcon_right
    pos_falcon_right = np.array([data.axes[0], data.axes[1], data.axes[2]])

def callback_left_box(data):
    global force_fb_left_box
    force_fb_left_box = np.array([data.wrench.force.x, data.wrench.force.y, data.wrench.force.z])

def callback_right_box(data):
    global force_fb_right_box
    force_fb_right_box = np.array([data.wrench.force.x, data.wrench.force.y, data.wrench.force.z])

def box_pos_cb(data):
    global pos_box, vel_box
    try:
        box_index = data.name.index("floating_box::base_link")
        pos_box[:] = [
            data.pose[box_index].position.x,
            data.pose[box_index].position.y,
            data.pose[box_index].position.z - 0.35
        ]
        vel_box[:] = [
            data.twist[box_index].linear.x,
            data.twist[box_index].linear.y,
            data.twist[box_index].linear.z
        ]
    except ValueError:
        rospy.logwarn_throttle(5, "floating_box::base_link not found in /gazebo/link_states")

# Subscribers
rospy.Subscriber("/falcon/joystick", Joy, callback_right_falcon)
rospy.Subscriber("/falcon/joystick1", Joy, callback_left_falcon)
rospy.Subscriber("/left/force_feedback", WrenchStamped, callback_left_box)
rospy.Subscriber("/right/force_feedback", WrenchStamped, callback_right_box)
rospy.Subscriber("/gazebo/link_states", LinkStates, box_pos_cb)

rate = rospy.Rate(100)
start_time = time.time()

try:
    while not rospy.is_shutdown():
        t = time.time() - start_time

        # Provide terminal guidance every 10 seconds
        phase = int(t // 10)
        instructions = [
            "Move FORWARD (+X)",
            "Move UPWARD (+Z)",
            "Move RIGHT (+Y)",
            "Move DOWNWARD (-Z)",
            "Move BACKWARD (-X)",
            "Move UPWARD again (+Z)",
            "Move LEFT (-Y)",
            "Move DOWN again (-Z)"
        ]
        if phase < len(instructions):
            rospy.loginfo_throttle(2, "[Guidance] " + instructions[phase])
        else:
            rospy.loginfo_throttle(2, "[Guidance] Return to start.")

        # Predefined 3D path
        if t < 10:
            xd, yd, zd = t / 40.0, 0.0, 0.0
        elif t < 20:
            xd, yd, zd = 0.25, 0.0, (t - 10) / 90.0
        elif t < 30:
            xd, yd, zd = 0.25, (t - 20) / 40.0, 0.11
        elif t < 40:
            xd, yd, zd = 0.25, 0.25, 0.11 - (t - 30) / 90.0
        elif t < 50:
            xd, yd, zd = 0.25 - (t - 40) / 40.0, 0.25, 0.0
        elif t < 60:
            xd, yd, zd = 0.0, 0.25, (t - 50) / 90.0
        elif t < 70:
            xd, yd, zd = 0.0, 0.25 - (t - 60) / 40.0, 0.11
        else:
            xd, yd, zd = 0.0, 0.0, 0.11 - (t - 70) / 90.0

        # Errors
        exl = pos_box[0] - pos_falcon_left[0] * 50
        eyl = pos_box[1] - (pos_falcon_left[2] - 0.12) * 50
        ezl = pos_box[2] - pos_falcon_left[1] * 50
        exr = pos_box[0] - pos_falcon_right[0] * 50
        eyr = pos_box[1] - (pos_falcon_right[2] - 0.12) * 50
        ezr = pos_box[2] - pos_falcon_right[1] * 50

        # Reduced gains
        uxlH = -3 * exl - 6 * vel_box[0]
        uylH = -2 * eyl - 6 * vel_box[1]
        uzlH = -4 * ezl - 1 * vel_box[2]
        uxrH = -3 * exr - 6 * vel_box[0]
        uyrH = -2 * eyr - 6 * vel_box[1]
        uzrH = -4 * ezr - 1 * vel_box[2]

        left_box(uxlH, uylH, uzlH)
        right_box(uxrH, uyrH, uzrH)
        left_falcon(-uxlH, -uzlH, -uylH)
        right_falcon(-uxrH, -uzrH, -uyrH)

        rate.sleep()

except rospy.ROSInterruptException:
    pass

