#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

                        ###note###
# ____________________________________________________
#|This is a simple counter.                           |
#|See accompanying flowchart for overall operation    |
#|This node is a combination of a subscriber          |
#|and publisher.                                      |
#|____________________________________________________|

import re
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Pose, Quaternion, PoseWithCovariance, TwistWithCovariance
from twist_to_motor_rps.msg import Num
from math import atan2, atan, pi, cos, sin, sqrt, pow, fabs, copysign


##########VARIABLES##########

#for the occupancy grid
occupancy_grid_callback_data = OccupancyGrid()

#for wheel encoder velocity reading:
wheel_encoder_velocity_callback_data= [0,0]

#for zed2 imu readings:
zed2_imu_callback_data = Imu()

#for publishing base velocity values in twist format
base_velocity = Twist()

ready= Bool()


#this callback function gets called whenever the occupancy grid is updated
def callback_occupancy_grid(data):
    global occupancy_grid_callback_data
    #print(data)
    #save to global variables
    occupancy_grid_callback_data = data
    '''map_resolution=data.info.resolution
    map_width=data.info.width
    map_height=data.info.height
    map_pose_x=data.info.origin.position.x
    map_pose_y=data.info.origin.position.y
    map_pose_z=data.info.origin.position.z
    map_ori_x=data.info.origin.orientation.x
    map_ori_y=data.info.origin.orientation.y
    map_ori_z=data.info.origin.orientation.z
    map_ori_w=data.info.origin.orientation.w
    map_cells=data.data
    print(map_resolution)
    print(map_width)
    print(map_height)
    print(map_pose_x)
    print(map_pose_y)
    print(map_pose_z)
    print(map_ori_x)
    print(map_ori_y)
    print(map_ori_z)
    print(map_ori_w)'''
    
def callback_wheel_encoder(data):
    global wheel_encoder_velocity_callback_data
    wheel_encoder_velocity_callback_data = data.num


    

def callback_odom_wheels(data):
    global odom_wheels_callback_data
    odom_wheels_callback_data = data




def callback_zed2_imu(data):
    global zed2_imu_callback_data
    zed2_imu_callback_data = data


def listener():
 

    

    global wheel_encoder_velocity_callback_data
    global zed2_imu_callback_data
    global ready
    rospy.loginfo("system_model started")
    ###Subscribers###


    #this call creates a subscriber and defines message type and which topic it publishes to
    #whenever a message is received it calls the callback function
    

    #actual wheel velocity from encoders subscriber:
    rospy.Subscriber('/encoder_rps', Num, callback_wheel_encoder)
    
    #zed2 imu subscriber:
    rospy.Subscriber("/zed2/zed_node/imu/data", Imu, callback_zed2_imu)

    #this value is a sleep value
    rate = rospy.Rate(5) #5Hz

#Robot parameters
    ts=2
    r=0.11 #radius of the wheel
    d=0.185 #distance between wheen and CG


#Setoperating variables
   
    xk=0
    yk=0
    phik=0
    time=0
    state_vector=[0,0,0]
    initial_state_vector=[0,0,0]

    #while ROS is not shutdown via terminal etc, run this in a loop at a rate of "rate Hz"
    while not rospy.is_shutdown():
   
        #Pull variables from callbacks so that they don't change during calculation 
      
        #wheel encoder velocity:
        wheel_encoder_velocity_left=wheel_encoder_velocity_callback_data[0]
        wheel_encoder_velocity_right=wheel_encoder_velocity_callback_data[1]
 

        #################################
        #insert navigation algorithm here
        #################################
        #solver for odometry
        
        rightvel=2*pi*r*wheel_encoder_velocity_right
        leftvel=2*pi*r*wheel_encoder_velocity_left

        #initial conditions
        if time==0:
            xk=initial_state_vector[0]
            yk=initial_state_vector[1]
            phik=initial_state_vector[2]


        #State space system and Euler Backward Integral Solver
        xkp=xk + ts*(r/2)*(rightvel+leftvel)*cos(phik)
        ykp=yk + ts*(r/2)*(rightvel+leftvel)*sin(phik)
        phikp=phik+ts*(r/(2*d))*(rightvel-leftvel)
        time=time+1

        #Reset heading angle for full rotation in both directions
        if fabs(phikp)>pi*2:
            phikp= fabs(phikp)-pi*2

        print("xk: "+ str(xk)+ " yk: "+str(yk)+ " phik: "+str(phik))

        #Set next values
        xk=xkp
        yk=ykp
        phik=phikp

        #publish xk yk phik
        state_vector=[xk,yk,phik]
        pub_state_vector.publish(state_vector)

 

        rate.sleep()

    #may not be needed when using "while not rospy.is_shutdown():"
    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()




if __name__ == '__main__':
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    #create a unique node:
    rospy.init_node('system_model', anonymous=True)
    #create a publisher object and defines which topic it subscribes to
    #pub = rospy.Publisher('progress', Int32, queue_size=10)

    pub_state_vector = rospy.Publisher('state_vector', Num, queue_size=10)

    #pub_timer_finished = rospy.Publisher('sentry_control_topic', String, queue_size=10)
    


    #pub_ready = rospy.Publisher('handshake_from_nav', Bool, queue_size=10, latch=True)
    #pub_command = rospy.Publisher('request_from_explore_sentry', String, queue_size=10)
    #pub_k_values = rospy.Publisher('xyphik', Num, queue_size=10)
    #this value is a sleep value
    rate = rospy.Rate(1) #5Hz

    #so node can register with master
    rate.sleep()

    #initialize this publisher
    #not ready initialy
    #ready = True
    #pub_ready.publish(ready)
    #start the subscribing and publishing process
    try:
        listener() 
    except rospy.ROSInterruptException:
        pass
    

