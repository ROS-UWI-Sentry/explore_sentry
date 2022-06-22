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

#for data from nav_sensor
nav_sensor_callback_data = [0, 0 ,0]

#for data from system_model
state_vector_data = [0, 0, 0]

#for data from control_act
control_act_data = "null"

commands_from_depth=Float64()

ready= Bool()
handshake_from_depth= Bool()

navigation_active_state="navigation_inactive"


def callback_nav_sensor(data):
    global nav_sensor_callback_data
    nav_sensor_callback_data = data.num


def callback_state_vector(data):
    global state_vector_data
    state_vector_data=data.num
    #print(state_vector_data)

def callback_control_act(data):
    global control_act_data
    control_act_data=data.data

def callback_navigation_control(data):
    global navigation_active_state
    navigation_active_state=data.data


def listener():
    global ready
    global nav_sensor_callback_data, state_vector_data
    global state_machine_data, control_act_data, navigation_active_state
    rospy.loginfo("state_machine started")
    ###Subscribers###


    #this call creates a subscriber and defines message type and which topic it publishes to
    #whenever a message is received it calls the callback function


    #data from nav_sensor:
    rospy.Subscriber('/nav_sensor_data', Num, callback_nav_sensor)

    #data from system model:
    rospy.Subscriber('/state_vector', Num, callback_state_vector)

    #data from control_act:
    rospy.Subscriber('/controls_act_to_state_machine', String, callback_control_act)

    #data from state_machine:
    rospy.Subscriber('/nav_control', String, callback_navigation_control)

    #this value is a sleep value
    rate = rospy.Rate(5) #5Hz

    ready = True
   # desired goals
    xdestraj=[0,2.5,2.5]
    ydestraj=[0,0,1]

    #xdestraj=[0,3]
    #ydestraj=[0,0]
    xd = 0
    yd = 0
    phid = 0

    cxd=0
    cyd=0
    cphid=0

    flag=-1
    localnavengaged=1 #no obsticle avoidance for demo
    iter=0

    #to know if the nextraj message is for the first point
    #so it wouldn't sanitize the starting point twice
    first_nextraj = True 


    #to know if sentry is moving between points or not
    #i.e. a point is already sent.
    in_motion = False

    #while ROS is not shutdown via terminal etc, run this in a loop at a rate of "rate Hz"
    while not rospy.is_shutdown():
   
        #nav sensor data:
        #nav_sensor = nav_sensor_callback_data
        #print("nav_sensor: "+str(nav_sensor))


        #################################
        #insert navigation algorithm here
        #################################
        #solver for odometry
        xk=state_vector_data[0]
        yk=state_vector_data[1]
        phik=state_vector_data[2]
        pub_to_nav_sensor.publish([xk,yk,phik])
        

        if navigation_active_state=="navigation_active":
            if not in_motion:
                #publish next goal, be in motion
                if iter<len(xdestraj)-1:
                    print("At goal, giving next goal")
                    iter=iter+1
                    xd=xdestraj[iter]
                    yd=ydestraj[iter]            
                    pub_to_control_act.publish([xd,yd,phid,flag])
                    in_motion = True   
                else:
                    pub_to_main_control.publish("all_goals_reached")
                    print("all goals reached")
            elif in_motion:
                #wait until next goal is requested
                if control_act_data=="NexTraj":
                    #tell fsm that the goal is reached
                    #to sanitize this spot
                    pub_to_main_control.publish("goal_reached")
                    #in the event there is a delay, inactivate nav
                    navigation_active_state="navigation_inactive"
        elif navigation_active_state=="navigation_inactive":
            in_motion=False     

   

        #print("xd: "+ str(xd)+ " yd: "+ str(yd)+ " phid: "+ str(phid) +" iter: "+ str(iter))
            #commands_from_depth_data=nav_sensor_callback_data
            #commands_from_depth_data=commands_from_depth.data  

        '''if commands_from_depth_data!=0: 
                ready=False #set that we are in local navigation
                phidkp=commands_from_depth_data+phik   

                print("Z: "+str(commands_from_depth_data)+" Dphi: "+ str(phidkp)+" Ephi: "+str(ephi)) 
'''
        #print(control_act_data)

        rate.sleep()

    #pub_to_main_control.Publish("goal_reached")
    #pub_to_main_control.Publish("all_goals_reached")
    #navigation_inactive
    #navigation_active

    

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
    rospy.init_node('state_machine', anonymous=True)
    #create a publisher object and defines which topic it subscribes to
    #pub = rospy.Publisher('progress', Int32, queue_size=10)

    pub_wheel_rps = rospy.Publisher('wheel_rps_vector', Num, queue_size=10)

    #pub_timer_finished = rospy.Publisher('sentry_control_topic', String, queue_size=10)
    


    #pub_ready = rospy.Publisher('handshake_from_nav', Bool, queue_size=10, latch=True)
    pub_to_nav_sensor = rospy.Publisher('state_machine_to_nav_sensor', Num, queue_size=10)
    pub_to_control_act = rospy.Publisher('state_machine_to_controls_act', Num, queue_size=10)
    pub_to_main_control = rospy.Publisher('sentry_control_topic', String, queue_size=10)
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
    

