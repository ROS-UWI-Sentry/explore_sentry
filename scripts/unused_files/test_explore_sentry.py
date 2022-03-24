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
from geometry_msgs.msg import Twist


        
#this function is for subscribing to messages
def listener():
 
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    #check for message on Topic

   
    rospy.loginfo("explore_sentry started")

    msg =Twist()
    msg.linear.x=0
    msg.linear.y=0
    msg.linear.z=0
    msg.angular.x=0  
    msg.angular.y=0       
    msg.angular.z=0  
    #this value is a sleep value
    rate = rospy.Rate(5) #1Hz
    
    while not rospy.is_shutdown() :
        msg.linear.x=0.1        
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()
        msg.linear.x=-0.1  
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()

if __name__ == '__main__':
    #create a unique node
    rospy.init_node('percentageHandler')
    #create a publisher object and defines which topic it subscribes to
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    
    
    #start the subscribing and publishing process
    try:
        listener()
        
    except rospy.ROSInterruptException:
        pass
    

