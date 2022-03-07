#!/usr/bin/env python

# This is a test node of the package. It prints out the position of the selected model.
# Last modified on Jan 7, 21 by Yubing Han


import rospy
from std_msgs.msg import String

def callback(data):
    print data.data
    
def listener2():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener2', anonymous=True)

    rospy.Subscriber("state_pub", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener2()
