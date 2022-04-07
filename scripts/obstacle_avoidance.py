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
#|This is uses point cloud                            |
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
from math import atan2, atan, pi, cos, sin, sqrt, pow, fabs, ceil

from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2


##########VARIABLES##########

#for the occupancy grid
occupancy_grid_callback_data = OccupancyGrid()

#for wheel encoder velocity reading:
wheel_encoder_velocity_callback_data= [0,0]

#for zed2 imu readings:
zed2_imu_callback_data = Imu()

#for publishing base velocity values in twist format
base_velocity = Twist()

#array for holding pointcloud
pntCldArr=[]


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

xyz=0

def callback_point_cloud(data):
    assert isinstance(data, PointCloud2)
    global xyz, pntCldArr
    count =0
    temp= 0.0
    
    #if xyz==1:

    gen = point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=False)
    cloud_points = []
    cloud_points = list(point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=False))
    print(cloud_points[434])	
    #print("using list: " + str(len(cloud_points)))   
    #print(len(gen))
    for p in gen:
            #print " X : %.2f Y: %.3f Z: %.3f" %(p[0],p[1],p[2])
            #tmpList=[p[0],p[1],p[2]]
            #pntCldArr.append(tmpList)
            #print(pntCldArr[count])
        count = count + 1

    print("count: " + str(count))
        #print("len: " + str(len(pntCldArr)))
	#print(pntCldArr)
	#print(tmpList)
    #xyz=xyz + 1




def listener():
 

    

    global occupancy_grid_callback_data, wheel_encoder_velocity_callback_data
    global odom_wheels_callback_data, zed2_imu_callback_data
    rospy.loginfo("obsctacle avoidance started")
    ###Subscribers###

    #this call creates a subscriber and defines message type and which topic it publishes to
    #whenever a message is received it calls the callback function
    

    #occupancy grid subscriber from rtabmap:
    rospy.Subscriber('/map', OccupancyGrid, callback_occupancy_grid)
    #actual wheel velocity from encoders subscriber:
    rospy.Subscriber('/encoder_rps', Num, callback_wheel_encoder)
    
    #zed2 imu subscriber:
    rospy.Subscriber("/zed2/zed_node/imu/data", Imu, callback_zed2_imu)
    
    #pointcloud subscriber:
    rospy.Subscriber("/zed2/zed_node/point_cloud/cloud_registered", PointCloud2, callback_point_cloud)
    
    #this value is a sleep value
    rate = rospy.Rate(5) #5Hz



    #Set reference positions


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
        
        #print(imu_lin_vel_z)

        #########################################
        #insert obstacle avoidance algorithm here
        #########################################
        robotwidth=5 #relative to ten centimerters
        robotlength=7 #relative to ten centimerters
        resolution=0.1
        #origin of robot
        xorigin=int(ceil(fabs(map_pose_x/resolution)))
        yorigin=int(ceil(fabs(map_pose_y/resolution)))
        #print(type(map_cells))
        #print(str(xorigin)+" : "+str(yorigin))
        #print(len(map_cells))
        point=(map_width*(23-1)+27)
        #print(point)
        #print(map_cells)
        #print( type(map_cells))
        #print(len(map_cells))
        #if (len(map_cells)>0):
        #    print(map_cells[0])
        #vision for 
        rowval=0
        colval=0
        frontvision=[]
        #for i in range(xorigin-5,xorigin):
        #    for j in range(yorigin,7+yorigin):
                #print(str(i)+","+str(j)+" " )
                #frontvision[rowval,colval]= 
                #print(map_cells[(])
                #colval=colval+1
            #colval=0
            #rowval=rowval+1

        #print(len(frontvision))
        #print(len(frontvision[0]))
            
        

        #print('xor: '+str(xcell)+'yor: '+str(ycell))

	#tempArr = []
	#tempArr[0]=9
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
    rospy.init_node('obstacle_avoidance', anonymous=True)
    #create a publisher object and defines which topic it subscribes to
    #pub = rospy.Publisher('progress', Int32, queue_size=10)

    pub_wheel_rps = rospy.Publisher('wheel_rps_vector', Num, queue_size=10)

    #pub_timer_finished = rospy.Publisher('sentry_control_topic', String, queue_size=10)
    
    #start the subscribing and publishing process
    try:
        listener() 
    except rospy.ROSInterruptException:
        pass
    

