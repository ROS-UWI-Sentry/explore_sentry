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

commands_from_depth=Float64()

ready= Bool()
handshake_from_depth= Bool()

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


def callback_data_from_depth(data):
    global commands_from_depth, ready
    commands_from_depth=data
    #print(commands_from_depth.data)
    #print("Inside receiving from depth")


'''
def callback_handshake_from_depth(data):
    global handshake_from_depth
    handshake_from_depth=data.data
    if handshake_from_depth==True:
        #print("callback_handshake_from_depth")
        #print(data.data)
        ready = True
        pub_ready.publish(ready)
        pub_command.publish("data from nav as string")'''



    #print(ready)



def listener():
 

    

    global occupancy_grid_callback_data, wheel_encoder_velocity_callback_data
    global odom_wheels_callback_data, zed2_imu_callback_data, commands_from_depth
    global ready
    rospy.loginfo("explore_sentry started")
    ###Subscribers###

    #this call creates a subscriber and defines message type and which topic it publishes to
    #whenever a message is received it calls the callback function
    

    #occupancy grid subscriber from rtabmap:
    rospy.Subscriber('/map', OccupancyGrid, callback_occupancy_grid)
    #actual wheel velocity from encoders subscriber:
    rospy.Subscriber('/encoder_rps', Num, callback_wheel_encoder)
    
    #zed2 imu subscriber:
    rospy.Subscriber("/zed2/zed_node/imu/data", Imu, callback_zed2_imu)
    
    #subscride to information from the zed_depth.cpp script
    rospy.Subscriber("commands_from_depth", Float64, callback_data_from_depth)

    #subscribe to handshake from the zed_depth.cpp script
    #rospy.Subscriber("handshake_from_depth", Bool, callback_handshake_from_depth)

    #this value is a sleep value
    rate = rospy.Rate(5) #5Hz

#Robot parameters
    ts=2
    r=0.11 #radius of the wheel
    d=0.185 #distance between wheen and CG

#Set gains  for controller
    alpha=0.1
    kp=0.1
    ki=0.001
    intek=0
    iter=0
    #xdestraj=[1.5,1.5,0.3,0,0] #x desired trajectories
    #ydestraj=[0,1.8,1.8,0,0] #y desired trajectories
    xdestraj=[1,2,2.5,2.5,2.5] #x desired trajectories
    ydestraj=[0,0,0,0,0] #y desired trajectories
    #Bounds to adjust
    xerrbound=0.2 #x range bound
    yerrbound=0.1 #y range bound
    ecludvec=0.17
    headingbound=0.3

#Setoperating variables
    yd=0
    xd=0
    wheel_rps=[0,0]
    xk=0
    yk=0
    phik=0
    time=0
    phidk=0
    exbound=0.000001
    eybound=0
    

    #while ROS is not shutdown via terminal etc, run this in a loop at a rate of "rate Hz"
    while not rospy.is_shutdown():
   
        #Pull variables from callbacks so that they don't change during calculation 

        #occupancy_grid information:
        map_resolution=occupancy_grid_callback_data.info.resolution
        map_width=occupancy_grid_callback_data.info.width
        map_height=occupancy_grid_callback_data.info.height
        #occupancy grid data:
        map_cells=occupancy_grid_callback_data.data
        #pose of map origin:
        #x,y,z position of map origin:
        map_pose_x=occupancy_grid_callback_data.info.origin.position.x
        map_pose_y=occupancy_grid_callback_data.info.origin.position.y
        map_pose_z=occupancy_grid_callback_data.info.origin.position.z
        #x,y,z,w quaternion of map origin:
        map_ori_x=occupancy_grid_callback_data.info.origin.orientation.x
        map_ori_y=occupancy_grid_callback_data.info.origin.orientation.y
        map_ori_z=occupancy_grid_callback_data.info.origin.orientation.z
        map_ori_w=occupancy_grid_callback_data.info.origin.orientation.w
        #end of pose of map origin
        
        
        #wheel encoder velocity:
        wheel_encoder_velocity_left=wheel_encoder_velocity_callback_data[0]
        wheel_encoder_velocity_right=wheel_encoder_velocity_callback_data[1]

        #zed2 imu data:
        imu_ori_x=zed2_imu_callback_data.orientation.x
        imu_ori_y=zed2_imu_callback_data.orientation.y
        imu_ori_z=zed2_imu_callback_data.orientation.z
        imu_ori_w=zed2_imu_callback_data.orientation.w
        imu_ang_vel_x=zed2_imu_callback_data.angular_velocity.x
        imu_ang_vel_y=zed2_imu_callback_data.angular_velocity.y
        imu_ang_vel_z=zed2_imu_callback_data.angular_velocity.z        
        imu_lin_vel_x=zed2_imu_callback_data.linear_acceleration.x
        imu_lin_vel_y=zed2_imu_callback_data.linear_acceleration.y
        imu_lin_vel_z=zed2_imu_callback_data.linear_acceleration.z

        #get data from zed_depth.cpp in this cycle:
        commands_from_depth_data=commands_from_depth.data
        
        
        #print(imu_lin_vel_z)

        #################################
        #insert navigation algorithm here
        #################################
        #solver for odometry
        
        rightvel=2*pi*r*wheel_encoder_velocity_right
        leftvel=2*pi*r*wheel_encoder_velocity_left
        #Grab coordinates
        xd=xdestraj[iter]
        yd=ydestraj[iter]
        #initial conditions
        if time==0:
            xk=0
            yk=0
            phik=0
            phidk=0
        #State space system and Euler Backward Integral Solver
        xkp=xk + ts*(r/2)*(rightvel+leftvel)*cos(phik)
        ykp=yk + ts*(r/2)*(rightvel+leftvel)*sin(phik)
        phikp=phik+ts*(r/(2*d))*(rightvel-leftvel)
        time=time+1

        #Reset heading angle for full rotation in both directions
        if fabs(phik)>pi*2:
            phikp=0
        #print("Left rps: "+ str(wheel_encoder_velocity_left) + "Right rps: "+ str(wheel_encoder_velocity_right))
        
       
         #Develop Control systems
        ex=xd-xk
        ey=yd-yk
        #error bounds to be within goal
        if fabs(ex)<=xerrbound:
            ex=exbound
            # ex=0.000001
        if fabs(ey)<=yerrbound:
            ey=eybound
            # ey=0
         
        vd=alpha*sqrt(pow(ex,2)+pow(ey,2))
        #phidkp=phidk+ 0.2*(atan2(ey,ex))
        phidkp=(atan2(ey,ex))
        if fabs(phidkp)>2*pi:
            phidkp=0

        print(commands_from_depth_data)
        phidkp=commands_from_depth_data
        #newang=0
        ''' if commands_from_depth!="":
            newang=re.findall(r"[-+]?(?:\d*\.\d+|\d+)", commands_from_depth_data)'''
        
        #newang=re.findall(r"[-+]?(?:\d*\.\d+|\d+)", commands_from_depth_data)
        #print(newang[0])
        vd=0

        ephi=phidk-phik
        intekp=intek+ki*ephi # integral part
        phicdot=kp*(ephi)+intekp
        
        #print("xdes: "+ str(xdestraj[iter])+ " xk: "+str(xk)+" zed xk: "+str(imu_ori_x)+ " ydes: "+ str(ydestraj[iter])+" yk: "+str(yk)+" zedyk: "+str(imu_ori_y)+" phides: "+ str(phidkp)+" phik: "+str(phik))

        #if using any loops consider adding "while not rospy.is_shutdown():" 
        #as a condition so that ctrl+C can break the loop
        if fabs(ephi)>ecludvec:
            vd=0 #stop moving and turn to correct heading
            
        #test rotation here
        
        #publish base velocity using individual rps:
        #left wheel rps:
        wl=(2*vd-phicdot*d)/(2*r)
        wr=(2*vd+phicdot*d)/(2*r)
        wheel_rps[0]=wl/(2*pi*r)
        #right wheel rps:
        wheel_rps[1]=wr/(2*pi*r)

        if sqrt(pow(ex,2)+pow(ey,2))<headingbound:
            wheel_rps[0]=0
            wheel_rps[1]=0

            #ready= True
            #pub_ready.publish(ready)

        #Get next trajectory
            if iter<len(xdestraj)-1:
                iter=iter+1        
        
        '''
        #if you are ready to accept data publish true:
        ready = True
        pub_ready.publish(ready)             
        
        #if you receive the okay to send data then send it:
        if handshake_from_depth==True:
            print("inside of sending to depth")
            ready = True
            pub_ready.publish(ready)
            pub_command.publish("data from nav as string")   

        '''

        #publish the rps to the wheels:
        pub_wheel_rps.publish(wheel_rps)

        #Set next values
        xk=xkp
        yk=ykp
        phik=phikp
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
    rospy.init_node('explore_sentry', anonymous=True)
    #create a publisher object and defines which topic it subscribes to
    #pub = rospy.Publisher('progress', Int32, queue_size=10)

    pub_wheel_rps = rospy.Publisher('wheel_rps_vector', Num, queue_size=10)

    #pub_timer_finished = rospy.Publisher('sentry_control_topic', String, queue_size=10)
    


    pub_ready = rospy.Publisher('handshake_from_nav', Bool, queue_size=10, latch=True)
    pub_command = rospy.Publisher('commands_from_nav', String, queue_size=10)
    #this value is a sleep value
    rate = rospy.Rate(1) #5Hz

    #so node can register with master
    rate.sleep()

    #initialize this publisher
    #not ready initialy
    ready = True
    pub_ready.publish(ready)
    #start the subscribing and publishing process
    try:
        listener() 
    except rospy.ROSInterruptException:
        pass
    

