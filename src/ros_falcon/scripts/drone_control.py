
import rospy,time
from geometry_msgs.msg import Twist,Wrench
from sensor_msgs.msg import Joy
from ros_falcon.msg import falconForces
import numpy as np

rospy.init_node("Box_control", anonymous=True) # Initialize ros node

pub_left_force_box = rospy.Publisher("/left_force", Twist, queue_size=1)
pub_left_force_falcon = rospy.Publisher("/falconForce", falconForces, queue_size=1)
pub_right_force_box = rospy.Publisher("/right_force", Twist, queue_size=1)
pub_right_force_falcon = rospy.Publisher("/falconForce1", falconForces, queue_size=1)

pos_falcon_left = np.array([0.0,0.0,0.0])
pos_falcon_right = np.array([0.0,0.0,0.0])

force_box_left = np.array([0.0,0.0,0.0])
force_box_right = np.array([0.0,0.0,0.0])

msg_box_force =  Wrench()
msg_falcon_force = falconForces()
joy_stick_sensitivity = 30

print("hi")

def pub_box_force(pub_force,force_x,force_y):
    msg_box_force.force.x = force_x
    msg_box_force.force.y = force_y
    pub_force.publish(msg_box_force)

def pub_falcon_force(pub_force,force_x,force_y):
    msg_falcon_force.X=force_x
    msg_falcon_force.Y=force_y
    pub_force.publish(msg_falcon_force)

def callback_pos_falcon_left(data):
    global pos_falcon_left
    pos_falcon_left = data.axes

def callback_pos_falcon_right(data):
    global pos_falcon_right
    pos_falcon_right = data.axes

def callback_force_box_left(data):
    global force_box_left
    force_box_left = data.force

def callback_force_box_right(data):
    global force_box_right
    force_box_right = data.force

rospy.Subscriber("/falcon/joystick", Joy, callback_pos_falcon_left,queue_size=1)
rospy.Subscriber("/left/force_feedback",Wrench,callback_force_box_left,queue_size=1)
rospy.Subscriber("/falcon/joystick1", Joy, callback_pos_falcon_right,queue_size=1)
rospy.Subscriber("/right/force_feedback",Wrench,callback_force_box_right,queue_size=1)

if __name__ == "__main__":
    try:
        program_starts = time.time()
        prev_time=0
        while not rospy.is_shutdown():
            if(time.time() - prev_time >= 0.01):
                prev_time=time.time()
                pos_falcon_left = joy_stick_sensitivity*pos_falcon_left
                pos_falcon_right = joy_stick_sensitivity*pos_falcon_right
                pub_box_force(pub_left_force_box,pos_falcon_left[0],pos_falcon_left[1]) 
                pub_box_force(pub_right_force_box,pos_falcon_right[0],pos_falcon_right[1])
                
                pub_falcon_force(pub_left_force_falcon,force_box_left[0],force_box_left[1])
                pub_falcon_force(pub_right_force_falcon,force_box_right[0],force_box_right[1])
                
    except KeyboardInterrupt:
        pass
