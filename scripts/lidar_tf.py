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

import roslib; roslib.load_manifest('laser_assembler')
import rospy; from laser_assembler.srv import *

import tf
angle = 0
br = tf.TransformBroadcaster()
def callback_angle(data):
    global angle
    angle=data.data
    br.sendTransform((0, 0, 0.585), #0.2m for ydlidar
    tf.transformations.quaternion_from_euler(radians(225-angle), 0, 0),
    rospy.Time.now(),
    "laser_frame",
    "base_footprint")
   



def listener():
    global angle
    rospy.loginfo("lidar_tf started")

    ###Subscribers###
    rospy.Subscriber('/angle', Int32, callback_angle) 

    while not rospy.is_shutdown():  
        try:
            assemble_scans = rospy.ServiceProxy('assemble_scans', AssembleScans)
            resp = assemble_scans(rospy.Time(0,0), rospy.get_rostime())
            print "Got cloud with %u points" % len(resp.cloud.points)
            pc_pub.publish(resp.cloud)
            rate.sleep()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


        

if __name__ == '__main__':     #Program start from here
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    #create a unique node:
    rospy.init_node('lidar_tf', anonymous=False)
    rospy.wait_for_service("assemble_scans")
    pc_pub = rospy.Publisher("point_cloud_combined", PointCloud)
    rate = rospy.Rate(5) #2Hz -> 500ms
    #so node can register with master
    rate.sleep()
    try:
        listener()
    except KeyboardInterrupt or rospy.ROSInterruptException:  # When 'Ctrl+C' is pressed, the program destroy() will be executed.
        destroy()
