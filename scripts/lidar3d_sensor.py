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




#the servo part of this code was taken from
#https://docs.sunfounder.com/projects/davinci-kit/en/latest/1.3.2_servo.html

##################VARIABLES##################
SERVO_MIN_PULSE = 500
SERVO_MAX_PULSE = 2500

ServoPin = 27

GPIO.setmode(GPIO.BCM)       # Numbers GPIOs by BCM
GPIO.setup(ServoPin, GPIO.OUT)   # Set ServoPin's mode is output
GPIO.output(ServoPin, GPIO.LOW)  # Set ServoPin to low
p = GPIO.PWM(ServoPin, 50)     # set Frequecy to 50Hz
p.start(0)


point_cloud_planar_temp = []

def callback_lida3d_sensor(data):
    global point_cloud_planar_temp
    #print(f"type of data: {type(data)}")
    point_cloud_planar_temp=data.points#[0].x
    #print(point_cloud_planar)
    

def map(value, inMin, inMax, outMin, outMax):
    return (outMax - outMin) * (value - inMin) / (inMax - inMin) + outMin

def setup():
    global p
    GPIO.setmode(GPIO.BCM)       # Numbers GPIOs by BCM
    GPIO.setup(ServoPin, GPIO.OUT)   # Set ServoPin's mode is output
    GPIO.output(ServoPin, GPIO.LOW)  # Set ServoPin to low
    p = GPIO.PWM(ServoPin, 50)     # set Frequecy to 50Hz
    p.start(0)                     # Duty Cycle = 0

def setAngle(angle):      # make the servo rotate to specific angle (0-180 degrees)
    angle = max(0, min(270, angle))
    pulse_width = map(angle, 0, 270, SERVO_MIN_PULSE, SERVO_MAX_PULSE)
    pwm = map(pulse_width, 0, 20000, 0, 100)
    #print("angle: "+ str(angle)+ "pwm: "+str(pwm))
    p.ChangeDutyCycle(pwm)#map the angle to duty cycle and output it
    print (str(pwm))

def loop():
    
    while True and not rospy.is_shutdown():
        #setAngle(10)
        #time.sleep(1)
        for i in range(200, 241, 1):   #make servo rotate from 0 to 180 deg
            setAngle(i)     # Write to servo
            print(i)
            time.sleep(0.001)
            
        time.sleep(1)
        for i in range(240, 199, -1): #make servo rotate from 180 to 0 deg
            setAngle(i)
            time.sleep(0.001)
            
        time.sleep(1)

def destroy():
    global p
    p.stop()
    GPIO.cleanup()

def listener():
    global point_cloud_planar_temp    
    rospy.loginfo("lidar3d_sensor started")
    max_servo_angle = 250
    min_servo_angle = 200
    ###Subscribers###
    rospy.Subscriber('/point_cloud', PointCloud, callback_lida3d_sensor) 
    #angle 225 is the real world "zero"
    waiting_time=0.1
    while not rospy.is_shutdown():
        #setAngle(225)
        #time.sleep(10)
        #top half
        #create new pointcloud object
        point_cloud = PointCloud()
        #fill header
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'laser_frame'
        point_cloud.header = header
        for angle in range (max_servo_angle, min_servo_angle-1, -1):   #make servo rotate from 251 to 200 deg
            pub_angle.publish(angle)
            setAngle(angle)
            time.sleep(waiting_time)
            print(angle)
            point_cloud_planar = point_cloud_planar_temp
            amount_of_points=len(point_cloud_planar)
            for point in range(amount_of_points):
                point_cloud_planar[point].z = tan(radians(angle-225))*sqrt(pow(point_cloud_planar[point].x,2)+pow(point_cloud_planar[point].y,2))
                #print(point_cloud_planar[point].z)
                point_cloud.points.append(point_cloud_planar[point])
        pc_pub.publish(point_cloud)  
        time.sleep(waiting_time)
        #create new pointcloud object
        point_cloud = PointCloud()
        #fill header
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'laser_frame'
        point_cloud.header = header

        #below half
        for angle in range (min_servo_angle, max_servo_angle+1, 1):   #make servo rotate from 200 to 251 deg
            pub_angle.publish(angle)
            setAngle(angle)
            time.sleep(waiting_time)
            print(angle)
            point_cloud_planar = point_cloud_planar_temp
            amount_of_points=len(point_cloud_planar)
            for point in range(amount_of_points):
                point_cloud_planar[point].z = tan(radians(angle-225))*sqrt(pow(point_cloud_planar[point].x,2)+pow(point_cloud_planar[point].y,2)) #225-angle
                point_cloud.points.append(point_cloud_planar[point])
        pc_pub.publish(point_cloud)
        time.sleep(waiting_time)

def pwm_rocker():
    global point_cloud_planar_temp    
    rospy.loginfo("lidar3d_sensor started")
    ###Subscribers###
    rospy.Subscriber('/point_cloud', PointCloud, callback_lida3d_sensor) 
    #angle 225 is the real world "zero"
    #initial position 
    i = 9.4 
    waiting_time=0.1
    p.ChangeDutyCycle(i)
    time.sleep(1)

    while not rospy.is_shutdown():
        #top half
        #create new pointcloud object
        point_cloud = PointCloud()
        #fill header
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'laser_frame'
        point_cloud.header = header
        while i < 12.2: #the value you want to go up to
            pub_angle.publish(angle)
            p.ChangeDutyCycle(i)
            time.sleep(waiting_time)
            point_cloud_planar = point_cloud_planar_temp
            amount_of_points=len(point_cloud_planar)
            for point in range(amount_of_points):
                point_cloud_planar[point].z = tan(radians(angle-225))*sqrt(pow(point_cloud_planar[point].x,2)+pow(point_cloud_planar[point].y,2))
                point_cloud.points.append(point_cloud_planar[point])            
            i = i + 0.4
        pc_pub.publish(point_cloud)  
        time.sleep(waiting_time)
        
        
        #bottom half
        point_cloud = PointCloud()
        #fill header
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'laser_frame'
        point_cloud.header = header
        while i > 9.4:
            pub_angle.publish(angle)
            p.ChangeDutyCycle(i)
            time.sleep(waiting_time)
            point_cloud_planar = point_cloud_planar_temp
            amount_of_points=len(point_cloud_planar)
            for point in range(amount_of_points):
                point_cloud_planar[point].z = tan(radians(angle-225))*sqrt(pow(point_cloud_planar[point].x,2)+pow(point_cloud_planar[point].y,2)) #225-angle
                point_cloud.points.append(point_cloud_planar[point])
            i = i - 0.4
        pc_pub.publish(point_cloud)
        time.sleep(waiting_time)




def laser_assembler():
    rospy.loginfo("lidar3d_sensor started")
    waiting_time=0.01
    while not rospy.is_shutdown():
        for angle in range (240, 199, -1):   #make servo rotate from 241 to 200 deg
            pub_angle.publish(angle)
            setAngle(angle)
            time.sleep(waiting_time)
            #print(angle)
        time.sleep(waiting_time)
        #below half
        for angle in range (200, 241, 1):   #make servo rotate from 200 to 241 deg
            pub_angle.publish(angle)
            setAngle(angle)
            time.sleep(waiting_time)
            #print(angle)
        time.sleep(waiting_time)



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
    pub_angle = rospy.Publisher('angle', Int32, queue_size=10)
    pc_pub = rospy.Publisher("point_cloud_combined", PointCloud, queue_size=100)
    rate_init.sleep()
    try:
        #loop()
        #listener()
        #laser_assembler()
        #p.ChangeDutyCycle(9.9)
        #time.sleep(2)
        #p.ChangeDutyCycle(11.1)
        #time.sleep(1)
        pwm_rocker()
        #destroy()

        p.stop()
        GPIO.cleanup()
        print("interrupt")
    

    except KeyboardInterrupt or rospy.ROSInterruptException:  # When 'Ctrl+C' is pressed, the program destroy() will be executed.
        pass
        #destroy()
        #p.stop()
        #GPIO.cleanup()
