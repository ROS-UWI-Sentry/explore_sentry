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


import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Pose, Quaternion, PoseWithCovariance, TwistWithCovariance
from twist_to_motor_rps.msg import Num



##########VARIABLES##########

#for the occupancy grid
occupancy_grid_callback_data = OccupancyGrid()

#for wheel encoder velocity reading:
wheel_encoder_velocity_callback_data= [0,0]

#for wheel odometry readings:
odom_wheels_callback_data = Odometry()

#for zed2 imu readings:
zed2_imu_callback_data = Imu()

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


#this callback function gets called whenever a message is recieved
#it pushes the message to the terminal for us to confirm what happened
#and it checks the value of the message and performs actions based on it
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + ' I heard: %s', data.data)
    global keepCounting, reset  
        
    if (data.data=="start_timer"): #this also continues timing
        keepCounting = True
        reset = False
        rospy.loginfo("started")
        #pub2.publish("timer_started")
        


def listener():
 
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    #check for message on Topic

    global occupancy_grid_callback_data, wheel_encoder_velocity_callback_data
    global odom_wheels_callback_data, zed2_imu_callback_data
    rospy.loginfo("explore_sentry started")
    ###Subscribers###

    #this call creates a subscriber and defines message type and which topic it publishes to
    #whenever a message is received it calls the callback function
    rospy.Subscriber('/timer_control_topic', String, callback)

    #occupancy grid subscriber from rtabmap:
    rospy.Subscriber('/map', OccupancyGrid, callback_occupancy_grid)
    #wheel velocity subscriber:
    rospy.Subscriber('/wheel_vel_vector', Num, callback_wheel_encoder)
    #wheel odometry subscriber:
    rospy.Subscriber("odom_wheels", Odometry, callback_odom_wheels)
    
    #zed2 imu subscriber:
    rospy.Subscriber("/zed2/zed_node/imu/data", Imu, callback_zed2_imu)
    
    
    #this value is a sleep value
    rate = rospy.Rate(1) #1Hz
    

    #while ROS is not shutdown via terminal etc, if the conditions are met, publish counting from 0 to 100
    i = 0
    t = 30
    while not rospy.is_shutdown():
   
        #Pull variables from callbacks
        #occupancy_grid:
        map_resolution=occupancy_grid_callback_data.info.resolution
        map_width=occupancy_grid_callback_data.info.width
        map_height=occupancy_grid_callback_data.info.height
        map_pose_x=occupancy_grid_callback_data.info.origin.position.x
        map_pose_y=occupancy_grid_callback_data.info.origin.position.y
        map_pose_z=occupancy_grid_callback_data.info.origin.position.z
        map_ori_x=occupancy_grid_callback_data.info.origin.orientation.x
        map_ori_y=occupancy_grid_callback_data.info.origin.orientation.y
        map_ori_z=occupancy_grid_callback_data.info.origin.orientation.z
        map_ori_w=occupancy_grid_callback_data.info.origin.orientation.w
        map_cells=occupancy_grid_callback_data.data
        
        #wheel encoder velocity:
        wheel_encoder_velocity_left=wheel_encoder_velocity_callback_data[0]
        wheel_encoder_velocity_right=wheel_encoder_velocity_callback_data[1]

        #odometry wheels data:
        odom_pose_x=odom_wheels_callback_data.pose.pose.position.x
        odom_pose_y=odom_wheels_callback_data.pose.pose.position.y
        odom_pose_z=odom_wheels_callback_data.pose.pose.position.z
        odom_ori_x=odom_wheels_callback_data.pose.pose.orientation.x
        odom_ori_x=odom_wheels_callback_data.pose.pose.orientation.y
        odom_ori_x=odom_wheels_callback_data.pose.pose.orientation.z
        odom_ori_x=odom_wheels_callback_data.pose.pose.orientation.w

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

        print(imu_lin_vel_z)



        rate.sleep()


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()




if __name__ == '__main__':
    #create a unique node
    rospy.init_node('percentageHandler', anonymous=True)
    #create a publisher object and defines which topic it subscribes to
    pub = rospy.Publisher('progress', Int32, queue_size=10)

    pub_timer_finished = rospy.Publisher('sentry_control_topic', String, queue_size=10)
    
    #start the subscribing and publishing process
    try:
        listener() 
    except rospy.ROSInterruptException:
        pass
    

