#!/usr/bin/env python
import time
import roslib
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from sensor_msgs.msg import PointCloud
from math import atan2, tan, atan, pi, cos, sin, sqrt, pow, fabs, copysign, radians

import tf
angle = 0
br = tf.TransformBroadcaster()
def callback_angle(data):
    global angle
    angle=data.data
    br.sendTransform((0, 0, 0),
    tf.transformations.quaternion_from_euler(radians(angle-225), 0, 0),
    rospy.Time.now(),
    "laser_frame",
    "base_footprint")



def listener():
    global angle
    rospy.loginfo("lidar_tf started")

    ###Subscribers###
    rospy.Subscriber('/angle', Int32, callback_angle) 

    while not rospy.is_shutdown():  
        '''if angle>199: 
            br.sendTransform((0, 0, 0),
            tf.transformations.quaternion_from_euler(0, (angle-255), 0),
            rospy.Time.now(),
            "laser_frame",
            "base_footprint")
        elif angle <=199:
            br.sendTransform((0, 0, 0),
            tf.transformations.quaternion_from_euler(0, (255-angle), 0),
            rospy.Time.now(),
            "laser_frame",
            "base_footprint")'''

        rate.sleep()


        

if __name__ == '__main__':     #Program start from here
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    #create a unique node:
    rospy.init_node('lidar_tf', anonymous=False)

    rate = rospy.Rate(2) #2Hz -> 500ms
    #so node can register with master
    rate.sleep()
    try:
        listener()
    except KeyboardInterrupt or rospy.ROSInterruptException:  # When 'Ctrl+C' is pressed, the program destroy() will be executed.
        destroy()
