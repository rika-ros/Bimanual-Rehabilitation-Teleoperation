#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState

# Global pose variables
model_pose_left = Pose()
model_pose_right = Pose()

# Publishers (initialized later)
pub_left = None
pub_right = None

def joy_callback_left(msg):
    global model_pose_left, pub_left

    # Wait until the publisher is connected
    while pub_left.get_num_connections() == 0 and not rospy.is_shutdown():
        rospy.loginfo_throttle(2, "[LEFT] Waiting for subscriber to /gazebo/set_model_state...")
        rospy.sleep(0.1)

    # Update pose based on joystick axes
    model_pose_left.position.x = msg.axes[0] - 0.2
    model_pose_left.position.y = msg.axes[1]
    model_pose_left.position.z = msg.axes[2]

    # Create and publish model state
    model_state = ModelState()
    model_state.model_name = "falcon_cursor_left"
    model_state.reference_frame = "world"
    model_state.pose = model_pose_left

    pub_left.publish(model_state)

def joy_callback_right(msg):
    global model_pose_right, pub_right

    # Wait until the publisher is connected
    while pub_right.get_num_connections() == 0 and not rospy.is_shutdown():
        rospy.loginfo_throttle(2, "[RIGHT] Waiting for subscriber to /gazebo/set_model_state...")
        rospy.sleep(0.1)

    # Update pose based on joystick axes
    model_pose_right.position.x = msg.axes[0] + 0.2
    model_pose_right.position.y = msg.axes[1]
    model_pose_right.position.z = msg.axes[2]

    # Create and publish model state
    model_state = ModelState()
    model_state.model_name = "falcon_cursor_right"
    model_state.reference_frame = "world"
    model_state.pose = model_pose_right

    pub_right.publish(model_state)

def control_model():
    global pub_left, pub_right

    rospy.init_node('control_model')

    pub_left = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
    pub_right = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)

    rospy.Subscriber("/falcon/joystick", Joy, joy_callback_left)
    rospy.Subscriber("/falcon/joystick1", Joy, joy_callback_right)

    rospy.loginfo("Joystick control for Falcon cursors started.")
    rospy.spin()

if __name__ == '__main__':
    control_model()

