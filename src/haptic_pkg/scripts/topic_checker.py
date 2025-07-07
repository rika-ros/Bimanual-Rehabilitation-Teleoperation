import rospy
from gazebo_msgs.msg import LinkStates

rospy.init_node("topic_checker")

def callback(data):
    print(data.pose[3].position)

rospy.Subscriber("/gazebo/link_states",LinkStates,callback,queue_size=1)

rospy.spin()