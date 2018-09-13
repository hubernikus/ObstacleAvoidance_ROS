#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from obstacle_recognition.msg import Obstacle

def callback(data):
    print('tataaaaa')
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.x0)
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.pose.position.x)

    
def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("object_2/pose", PoseStamped, callback)
    #rospy.Subscriber("obstacle1", Obstacle, callback)

    # spin() simply keeps python from exiting until this node is stopped
    print('spining')
    rospy.spin()

    print('ended')
    

if __name__ == '__main__':
    listener()
