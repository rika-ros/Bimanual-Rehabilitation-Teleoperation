#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import Joy
import numpy as np

class AssistAsNeededController:
    def __init__(self):
        rospy.init_node('aan_controller')

        # Assistance gain parameters
        self.assist_gain = 5.0        # how strong the assist should be
        self.user_effort_threshold = 0.15  # deadzone below which we consider the user is weak

        # Joystick state
        self.user_input = np.zeros(3)  # [x, y, z]

        # Publisher to assistive force topic (right arm for now)
        self.force_pub = rospy.Publisher('/right/force', Wrench, queue_size=10)

        # Subscribe to the right Falcon joystick
        rospy.Subscriber('/falcon/joystick1', Joy, self.joystick_callback)

    def joystick_callback(self, msg):
        self.user_input[0] = msg.axes[0]
        self.user_input[1] = msg.axes[1]
        self.user_input[2] = msg.axes[2]

    def assist_logic(self):
        assist_force = np.zeros(3)
        for i in range(3):
            if abs(self.user_input[i]) < self.user_effort_threshold:
                assist_force[i] = self.assist_gain * np.sign(self.user_input[i])
            else:
                assist_force[i] = 0.0
        return assist_force

    def run(self):
        rate = rospy.Rate(100)  # 100 Hz
        while not rospy.is_shutdown():
            force = self.assist_logic()

            wrench = Wrench()
            wrench.force.x = force[0]
            wrench.force.y = force[1]
            wrench.force.z = force[2]

            self.force_pub.publish(wrench)
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = AssistAsNeededController()
        controller.run()
    except rospy.ROSInterruptException:
        pass

