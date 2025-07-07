#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Wrench
from gazebo_msgs.msg import ModelStates
import time
from sensor_msgs.msg import Joy

class BoxPIDController:
    def __init__(self):
        self.kp = 1.1
        self.ki = 0
        self.kd = 7
        self.i_term = 0.0
        self.last_error_x = 0.0
        self.last_error_y = 0.0
        self.last_error_z = 0.0
        self.force_pub = rospy.Publisher('/right/force', Wrench, queue_size=10)
        self.pose_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.pose_callback)
        self.pose_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.pose_callback)
        self.pose_falcon_sub = rospy.Subscriber("/falcon/joystick1", Joy, self.callback_pos_falcon_right,queue_size=1)
        self.desired_position_x = 0
        self.desired_position_y = 0
        self.desired_position_z = 0.1
        self.position_x = 0
        self.position_y = 0
        self.position_z = 0

    def pose_callback(self, msg):
        # Extract position of box from the message
        index = msg.name.index("falcon_cursor_right")
        self.position_x = msg.pose[index].position.x
        self.position_y = msg.pose[index].position.y
        self.position_z = msg.pose[index].position.z
    
    def callback_pos_falcon_right(self,data):
        self.desired_position_x = data.axes[0] + 0.225
        self.desired_position_y = data.axes[1]
        self.desired_position_z = data.axes[2]
        # print(self.desired_position_x,self.desired_position_y,self.desired_position_z)

    def pid_pub(self):
        # Compute error and update integral term
        error_x = self.desired_position_x - self.position_x
        error_y = self.desired_position_y - self.position_y
        error_z = self.desired_position_z - self.position_z
        # self.i_term = min(max(self.i_term, -100), 100) # saturation

        # Compute derivative term
        

        # Compute PID output
        output_x = self.kp * error_x + self.kd * (error_x - self.last_error_x)
        self.last_error_x = error_x

        output_y = self.kp * error_y + self.kd * (error_y - self.last_error_y)
        self.last_error_y = error_y

        output_z = self.kp * error_z + self.kd * (error_z - self.last_error_z)
        self.last_error_z = error_z
        # Create wrench message and set force
        # print(error_x,error_y)
        wrench_msg = Wrench()
        wrench_msg.force.x = output_x
        wrench_msg.force.y = output_y
        wrench_msg.force.z = output_z

        # Publish the force
        self.force_pub.publish(wrench_msg)

if __name__ == '__main__':
    rospy.init_node('box_pid_controller')
    # time.sleep(5)
    controller = BoxPIDController()

    try:
        program_starts = time.time()
        prev_time=0
        while not rospy.is_shutdown():
            if(time.time() - prev_time >= 0.01):
                prev_time=time.time()
                controller.pid_pub()
    except:
        pass