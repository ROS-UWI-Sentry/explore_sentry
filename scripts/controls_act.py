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

state_machine_data=[0,0,0,0]
system_model_data=[0,0,0]


ready= Bool()
handshake_from_depth= Bool()

    
def callback_wheel_encoder(data):
    global wheel_encoder_velocity_callback_data
    wheel_encoder_velocity_callback_data = data.num
    
   


def callback_data_from_state_machine(data):
    global state_machine_data
    state_machine_data = data.num


def callback_data_from_system_model(data):
    global system_model_data
    system_model_data = data.num



def callback_zed2_imu(data):
    global zed2_imu_callback_data
    zed2_imu_callback_data = data



def listener():
 

    

    global occupancy_grid_callback_data, wheel_encoder_velocity_callback_data
    global odom_wheels_callback_data, zed2_imu_callback_data
    global ready
    global state_machine_data, system_model_data
    rospy.loginfo("controls_act started")
    ###Subscribers###


    #this call creates a subscriber and defines message type and which topic it publishes to
    #whenever a message is received it calls the callback function
    


    #actual wheel velocity from encoders subscriber:
    rospy.Subscriber('/encoder_rps', Num, callback_wheel_encoder)
    
    #zed2 imu subscriber:
    rospy.Subscriber("/zed2/zed_node/imu/data", Imu, callback_zed2_imu)


    #subscribe to information from state_machine
    rospy.Subscriber("state_machine_to_controls_act", Num, callback_data_from_state_machine)

    #subscribe to state_vector from sytem_model
    rospy.Subscriber("state_vector", Num, callback_data_from_system_model)    

    #subscribe to handshake from the zed_depth.cpp script
    #rospy.Subscriber("handshake_from_depth", Bool, callback_handshake_from_depth)

    #this value is a sleep value
    rate = rospy.Rate(5) #5Hz




#Set gains  for controller
    alpha=0.1
    kp=0.1
    ki=0.001
    intek=0

    #Bounds to adjust
    xerrbound=0.15 #x range bound
    yerrbound=0.1 #y range bound
    ecludvec=0.15    #0.17 angle bound
    headingbound=0.1 #0.3 

#Setoperating variables
    yd=0
    xd=0
    xk=0
    yk=0
    wheel_rps=[0,0]
    phik=0
    time=0
    phidk=0
    phidkp=0
    exbound=0.000001
    eybound=0
    flag=0

    #des_current_traj=[xd,yd,phik,xk,yk,phik,flag]


#Robot parameters
    ts=2
    r=0.11 #radius of the wheel
    d=0.185 #distance between wheen and CG

    #while ROS is not shutdown via terminal etc, run this in a loop at a rate of "rate Hz"
    while not rospy.is_shutdown():
   
        #Pull variables from callbacks so that they don't change during calculation 
       
        
        #print(imu_lin_vel_z)

        #################################
        #insert navigation algorithm here
        #################################  

        xd=state_machine_data[0]
        yd=state_machine_data[1]
        #phid=state_machine_data[2]
        flag=state_machine_data[3]
        xk=system_model_data[0]
        yk=system_model_data[1]
        phik=system_model_data[2]
        ''' xd=des_current_traj=[0]
        yd=des_current_traj=[1]
        phik=des_current_traj=[2]
        xk=des_current_traj=[3]
        yk=des_current_traj=[4]
        phik=des_current_traj=[5]'''

        
        #print("Left rps: "+ str(wheel_encoder_velocity_left) + "Right rps: "+ str(wheel_encoder_velocity_right))
          


        #Develop Errors for controls
        ex=xd-xk
        ey=yd-yk

        

        #error bounds to be within goal
        if fabs(ex)<=xerrbound:
            ex=exbound
        if fabs(ey)<=yerrbound:
            ey=eybound

        #Get velocity and heading references
        vd=alpha*sqrt(pow(ex,2)+pow(ey,2))
        phidk=(atan2(ey,ex))
        if phidk<0 and phik>0:
            phidk=atan2(ey,ex)+2*pi


        ephi=phidk-phik
        intekp=intek+ki*ephi # integral part
        phicdot=kp*(ephi)+intekp
        
        

        #error heading is out of phi bound
        if fabs(ephi)>ecludvec:
            vd=0 #stop moving and turn to correct heading
            print("phidk: "+ str(phidk)+" phi: "+str(phik))

        else:
            print("xd: "+ str(xd) +" yd: "+ str(yd) +" xk: "+str(xk)+ " yk: "+str(yk))

            
        #test rotation here
        
        #publish base velocity using individual rps:
        #left wheel rps:
        wl=(2*vd-phicdot*d)/(2*r)
        wr=(2*vd+phicdot*d)/(2*r)
        wheel_rps[0]=wl/(2*pi*r)
        #right wheel rps:
        wheel_rps[1]=wr/(2*pi*r)

        #We are within a bound of the desired
        if sqrt(pow(ex,2)+pow(ey,2))<headingbound:
            #Stop and get next trajectory
            wheel_rps[0]=0
            wheel_rps[1]=0
            pub_to_state_machine.publish("NexTraj")
            print("Next Traj")
            


        #publish the rps to the wheels:
        pub_wheel_rps.publish(wheel_rps)

        phidk=phidkp
       


        '''#twist_to_motor node converts this into individual wheel velocities 
        base_velocity.linear.x=vd #velocity in x direction
        base_velocity.angular.z=phicdot #velocity around z
        pub_base_velocity.publish(base_velocity)'''


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
    rospy.init_node('controls_act', anonymous=True)
    #create a publisher object and defines which topic it subscribes to
    #pub = rospy.Publisher('progress', Int32, queue_size=10)

    pub_wheel_rps = rospy.Publisher('wheel_rps_vector', Num, queue_size=10)

    #pub_timer_finished = rospy.Publisher('sentry_control_topic', String, queue_size=10)
    

    pub_to_state_machine = rospy.Publisher('controls_act_to_state_machine', String, queue_size=10)
    #this value is a sleep value
    rate = rospy.Rate(1) #5Hz

    #so node can register with master
    rate.sleep()

    #initialize this publisher

    #start the subscribing and publishing process
    try:
        listener() 
    except rospy.ROSInterruptException:
        pass
    

