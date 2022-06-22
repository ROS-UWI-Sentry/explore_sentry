#!/usr/bin/env python
import RPi.GPIO as GPIO
import time
import smbus            #import SMBus module of I2C

#ros imports
import roslib
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud
from math import atan2, tan, atan, pi, cos, sin, sqrt, pow, fabs, copysign, radians

from twist_to_motor_rps.msg import Num

import roslib; roslib.load_manifest('laser_assembler')
import rospy; from laser_assembler.srv import *
import tf

import threading
 
br = tf.TransformBroadcaster()

#setup servo
ServoPin = 27
GPIO.setmode(GPIO.BCM)       # Numbers GPIOs by BCM
GPIO.setup(ServoPin, GPIO.OUT)   # Set ServoPin's mode is output
GPIO.output(ServoPin, GPIO.LOW)  # Set ServoPin to low
p = GPIO.PWM(ServoPin, 50)     # set Frequecy to 50Hz
p.start(0)


point_cloud_temp = sensor_msgs.msg._PointCloud.PointCloud()


def laser_assembler():
    global point_cloud_temp
    rospy.loginfo("lidar3d_sensor started")
    #initial position 
    i = 9.4 
    waiting_time=0.5
    p.ChangeDutyCycle(i)
    time.sleep(1)
    pwmArray = [9.4, 9.8, 10.2, 10.6, 11, 11.4, 11.8, 12.2]
    #pwmArray = [11, 11, 11, 11, 11, 11, 11, 11]
    angleArray = [-42.5, -32.5, -22.5, -10, 0, 10, 20, 32.5]
    while not rospy.is_shutdown(): 
        for x in range(7):
            
            angle = angleArray[x]
            pwm = pwmArray[x]         
            #pub_angle.publish(radians(angle))
            p.ChangeDutyCycle(pwm)
            time.sleep(waiting_time)
            #send tf
            br.sendTransform((0, 0, 0.585), #0.2m for ydlidar
            tf.transformations.quaternion_from_euler(-radians(angle), 0, 0),
            rospy.Time.now(),
            "laser_frame",
            "base_footprint")  
            #get a pointcloud and publish it
            try:               
                assemble_scans = rospy.ServiceProxy('assemble_scans', AssembleScans)
                resp = assemble_scans(rospy.Time(0,0), rospy.get_rostime())
                print "Got cloud with %u points" % len(resp.cloud.points)
                point_cloud_temp=resp.cloud
                pc_pub.publish(point_cloud_temp)
                rate.sleep()
            except rospy.ServiceException, e:  
                print "Service call failed: %s"%e                 

        for x in range(7, -1, -1): #to decrement
            
            angle = angleArray[x]
            pwm = pwmArray[x]
            #pub_angle.publish(radians(angle))
            p.ChangeDutyCycle(pwm)
            time.sleep(waiting_time)
            #send tf
            br.sendTransform((0, 0, 0.585), #0.2m for ydlidar
            tf.transformations.quaternion_from_euler(-radians(angle), 0, 0),
            rospy.Time.now(),
            "laser_frame",
            "base_footprint")  
            #get a pointcloud for one sweep and publish it
            try:
                assemble_scans = rospy.ServiceProxy('assemble_scans', AssembleScans)
                resp = assemble_scans(rospy.Time(0,0), rospy.get_rostime())
                print "Got cloud with %u points" % len(resp.cloud.points)
                point_cloud_temp=resp.cloud
                pc_pub.publish(point_cloud_temp)
                rate.sleep()
            except rospy.ServiceException, e: 
                print "Service call failed: %s"%e

def test_without_laser_assembler():
    global point_cloud_temp
    rospy.loginfo("lidar3d_sensor started")
    #initial position 
    i = 9.4 
    waiting_time=0.1
    p.ChangeDutyCycle(i)
    time.sleep(1)
    pwmArray = [9.4, 9.8, 10.2, 10.6, 11, 11.4, 11.8, 12.2]
    #pwmArray = [11, 11, 11, 11, 11, 11, 11, 11]
    angleArray = [-42.5, -32.5, -22.5, -10, 0, 10, 20, 32.5]
    while not rospy.is_shutdown(): 
        for x in range(7):
            
            angle = angleArray[x]
            pwm = pwmArray[x]         
            pub_angle.publish(radians(angle))
            p.ChangeDutyCycle(pwm)
            time.sleep(waiting_time)

            #send tf
            br.sendTransform((0, 0, 0.585), #0.2m for ydlidar
            tf.transformations.quaternion_from_euler(0, 0, 0),
            rospy.Time.now(),
            "laser_frame",
            "base_footprint")  
                 
        for x in range(7, -1, -1): #to decrement
            
            angle = angleArray[x]
            pwm = pwmArray[x]
            pub_angle.publish(radians(angle))
            p.ChangeDutyCycle(pwm)
            time.sleep(waiting_time)

            #send tf
            br.sendTransform((0, 0, 0.585), #0.2m for ydlidar
            tf.transformations.quaternion_from_euler(0, 0, 0),
            rospy.Time.now(),
            "laser_frame",
            "base_footprint")  
    
        



def process_pointcloud():
    global point_cloud_temp
    while not rospy.is_shutdown():
        print "thread got cloud with %u points" % len(point_cloud_temp.points)
        time.sleep(1)
    


if __name__ == '__main__':     #Program start from here
    #setup()
    GPIO.setmode(GPIO.BCM)       # Numbers GPIOs by BCM
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    #create a unique node:
    rospy.init_node('state_machine', anonymous=False)

    #move lidar to topmost angle:
    #setAngle(251)
    #p.ChangeDutyCycle(9.5) #
    #this value is a sleep value
    rate_init = rospy.Rate(1) #1Hz
    rate = rospy.Rate(10) #2Hz -> 500ms
    #so node can register with master
    pub_angle = rospy.Publisher('angle', Float64, queue_size=10)
    pc_pub = rospy.Publisher("point_cloud_combined", PointCloud, queue_size=100)

    #change topic names to better reflect node name
    pub_commands = rospy.Publisher("handshake_from_depth", Float64, queue_size=100)
    pub_lidar3d_ready = rospy.Publisher("handshake_from_depth", Bool, queue_size=100)
    pub_to_state_machine = rospy.Publisher("nav_sensor_data", Num, queue_size=100)
    rate_init.sleep()
    try:
        test_without_laser_assembler()

        #thread = threading.Thread(target=process_pointcloud, args=())
        #thread.start()

        #laser_assembler()
        

        p.stop()
        GPIO.cleanup()
        print("interrupt")
    

    except KeyboardInterrupt or rospy.ROSInterruptException:  # When 'Ctrl+C' is pressed, the program destroy() will be executed.
        pass
        #destroy()
        #p.stop()
        #GPIO.cleanup()
