#!/usr/bin/env python

# This node: 1) subscribe to the name publisher and gazebo model state service; 2) publish the name and position
# Last modified on Jan 7, 21 by Yubing Han


import rospy 
from std_msgs.msg import String
from gazebo_msgs.srv import GetModelState 

pub=rospy.Publisher('state_pub',String, queue_size=1000)

def getCoord(data):
    nm = data.data.strip('\"')
    
    try: 
        model_coordinates = rospy.ServiceProxy( '/gazebo/get_model_state', GetModelState)
        object_coordinates = model_coordinates(nm, "")
        z_position = object_coordinates.pose.position.z
        y_position = object_coordinates.pose.position.y
        x_position = object_coordinates.pose.position.x
        st = nm + "\nx: "+ str(x_position) + "\ny: " + str(y_position) + "\nz: "+ str(z_position)
        print st
        pub.publish(st)

    except rospy.ServiceException as e:
        rospy.loginfo("Get model state service call failed: {0}".format(e))

def getName():
    rospy.init_node('getName', anonymous=True)

    rospy.Subscriber("mod_name", String, getCoord)

    rospy.spin()

if __name__ == '__main__':
    getName()
