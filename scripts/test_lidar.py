#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time

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

##################VARIABLES##################

ServoPin = 24


#init
GPIO.setmode(GPIO.BCM)       # Numbers GPIOs by BCM
GPIO.setup(ServoPin, GPIO.OUT)   # Set ServoPin's mode is output
GPIO.output(ServoPin, GPIO.LOW)  # Set ServoPin to low
p = GPIO.PWM(ServoPin, 50)     # set Frequecy to 50Hz
p.start(0)                     # Duty Cycle = 0






def listener():
    global p
    rospy.loginfo("lidar3d_sensor started")

    ###Subscribers###

    while not rospy.is_shutdown():
        p.ChangeDutyCycle(50)
        #time.sleep(1)
        rate.sleep()

    p.stop()
    GPIO.cleanup()


if __name__ == '__main__':     #Program start from here
    #setup()
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    #create a unique node:
    rospy.init_node('lidar3d_sensor', anonymous=False)

    #move lidar to topmost angle:
    #setAngle(251)
    #this value is a sleep value
    rate_init = rospy.Rate(1) #1Hz
    rate = rospy.Rate(10) #2Hz -> 500ms
    #so node can register with master
    pub_angle = rospy.Publisher('angle', Int32, queue_size=10)
    pc_pub = rospy.Publisher("point_cloud_combined", PointCloud)
    rate_init.sleep()
    #time.sleep(1)
    try:
        
        listener()
        #p.ChangeDutyCycle(50)
        #time.sleep(3)
        #p.stop()
        #GPIO.cleanup()
    except KeyboardInterrupt: #rospy.ROSInterruptException:  # When 'Ctrl+C' is pressed, the program destroy() will be executed.
        print("inside interrupt")
        p.stop()
        GPIO.cleanup()


