#!/usr/bin/env python3
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

#######imu variables#######
#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47


def MPU_Init():
    #write to sample rate register
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
     
    #Write to power management register
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
     
    #Write to Configuration register
    bus.write_byte_data(Device_Address, CONFIG, 0)
     
    #Write to Gyro configuration register
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
     
    #Write to interrupt enable register
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)
def read_raw_data(addr):
    #Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)
     
        #concatenate higher and lower value
        value = ((high << 8) | low)
         
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value
 
#setup servo
ServoPin = 27
GPIO.setmode(GPIO.BCM)       # Numbers GPIOs by BCM
GPIO.setup(ServoPin, GPIO.OUT)   # Set ServoPin's mode is output
GPIO.output(ServoPin, GPIO.LOW)  # Set ServoPin to low
p = GPIO.PWM(ServoPin, 50)     # set Frequecy to 50Hz
p.start(0)

#setup imu
#on the pinout this is given as I2C bus 0 
#command to see if device is connected:
#sudo i2cdetect -y -r 8
#expect to see 68 on row 60 column 8 
bus = smbus.SMBus(8)    # or bus = smbus.SMBus(0) for older version boards
#sleep(1)
Device_Address = 0x68   # MPU6050 device address
 
MPU_Init()

point_cloud_planar_temp = []

def callback_lida3d_sensor(data):
    global point_cloud_planar_temp
    #print(f"type of data: {type(data)}")
    point_cloud_planar_temp=data.points#[0].x
    #print(point_cloud_planar)


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

def readGyro():
    #Read Gyroscope raw value
    gyro_x = read_raw_data(GYRO_XOUT_H)
    Gx = gyro_x/131.0
    return Gx

def pwm_rocker():
    global point_cloud_planar_temp    
    rospy.loginfo("lidar3d_sensor started")
    ###Subscribers###
    rospy.Subscriber('/point_cloud', PointCloud, callback_lida3d_sensor) 
    
    #initial position 
    i = 9.4 
    waiting_time=0.1
    p.ChangeDutyCycle(i)
    time.sleep(1)
    pwmArray = [9.4, 9.8, 10.2, 10.6, 11, 11.4, 11.8, 12.2]
    angleArray = [-42.5, -32.5, -22.5, -10, 0, 10, 20, 32.5]
    while not rospy.is_shutdown(): 
        #top half
        #create new pointcloud object
        point_cloud = PointCloud()
        #fill header
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'laser_frame'
        point_cloud.header = header

       
        for x in range(7):
        #while i < 12.2: #the value you want to go up to
            angle = angleArray[x]
            pwm = pwmArray[x]
            pub_angle.publish(angle)
            p.ChangeDutyCycle(pwm)
            time.sleep(waiting_time)

            point_cloud_planar = point_cloud_planar_temp
            amount_of_points=len(point_cloud_planar)
            for point in range(amount_of_points):
                point_cloud_planar[point].z = tan(radians(angle))*sqrt(pow(point_cloud_planar[point].x,2)+pow(point_cloud_planar[point].y,2))
                point_cloud.points.append(point_cloud_planar[point])            
        pc_pub.publish(point_cloud)  
        time.sleep(waiting_time)
        
        
        #bottom half
        point_cloud = PointCloud()
        #fill header
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'laser_frame'
        point_cloud.header = header

        for x in range(7, -1, -1): #to decrement
        #while i > 9.4:
            angle = angleArray[x]
            pwm = pwmArray[x]
            pub_angle.publish(angle)
            p.ChangeDutyCycle(pwm)
            time.sleep(waiting_time)

            point_cloud_planar = point_cloud_planar_temp
            amount_of_points=len(point_cloud_planar)
            for point in range(amount_of_points):
                point_cloud_planar[point].z = tan(radians(angle))*sqrt(pow(point_cloud_planar[point].x,2)+pow(point_cloud_planar[point].y,2))
                point_cloud.points.append(point_cloud_planar[point])
        pc_pub.publish(point_cloud)
        time.sleep(waiting_time)

        #time.sleep(1)




def laser_assembler():
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
            pub_angle.publish(angle)
            p.ChangeDutyCycle(pwm)
            time.sleep(waiting_time)

        for x in range(7, -1, -1): #to decrement
            angle = angleArray[x]
            pwm = pwmArray[x]
            pub_angle.publish(angle)
            p.ChangeDutyCycle(pwm)
            time.sleep(waiting_time)


    '''waiting_time = 0.01
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
        time.sleep(waiting_time)'''



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
    #pc_pub = rospy.Publisher("point_cloud_combined", PointCloud, queue_size=100)
    rate_init.sleep()
    try:
        #loop()
        #listener()
        laser_assembler()
        #p.ChangeDutyCycle(9.9)
        #time.sleep(2)
        #p.ChangeDutyCycle(11.1)
        #time.sleep(1)
        #pwm_rocker()
        #destroy()

        p.stop()
        GPIO.cleanup()
        print("interrupt")
    

    except KeyboardInterrupt or rospy.ROSInterruptException:  # When 'Ctrl+C' is pressed, the program destroy() will be executed.
        pass
        #destroy()
        #p.stop()
        #GPIO.cleanup()
