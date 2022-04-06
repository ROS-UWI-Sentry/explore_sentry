///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2018, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

/**
 * This tutorial demonstrates simple receipt of ZED depth messages over the ROS system.
 */

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sstream>
#include <tgmath.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"

int global_run_once = 0;
std_msgs::Float64 msg_angle;
ros::Publisher commands;
std_msgs::String msg_string;

//std_msgs::Bool nav_ready;
bool nav_ready;

 


/**
 * Subscriber callback
 */

float scandepth(int left_col, int u, int width, int height, float depthbound,float *depths){
  float distance_count=0;

  for(int i=left_col; i!=u+left_col && ros::ok();i++){
      //iterate through width space
      for (int q=0;q!=height-1 && ros::ok();q++){
        //iterate through top right quadrant
        int pixval=i+width*q;
        //std::cout <<"pixel sel: "<<pixval<<'\n';
        //ROS_INFO("%g",msg->width);
        float depth=depths[pixval];
        //float depth=1;
        if (depth<depthbound){
            distance_count=distance_count+1;
            //std::cout <<"x: "<< q << " " << "y: " << i << " " <<"pixval: " << pixval << " " <<"distance: " << depth<< '\n';
        }
        
      }
       
    }

  float column_area=u*height;
    float perc_covered=distance_count/column_area;
    return perc_covered;
}

void handshakeCallback(const std_msgs::Bool::ConstPtr& msg)
{
  nav_ready=&msg->data;
  std::cout <<"HANDSHAKE VALUE:"<<nav_ready<<'\n';
}
  



void depthCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  


  
  // Get a pointer to the depth values casting the data
  // pointer to floating point
  float* depths = (float*)(&msg->data[0]);

  // Image coordinates of the center pixel
  //int u = msg->width / 2;
  //int v = msg->height / 2;

  // Linear index of the center pixel
  //int centerIdx = u + msg->width * v;
  //int alpha=4; //width aspect must be greater than 2
  //int beta=3; //height aspect must be greater than 2
  

  int u=388; //robot width at 1 meter perspective projection
  int v=357; //robot height at 1 meter perspective projection

int leftwidtharr[6]={164,358,552,746,910,940};
int rightwidtharr[6]={1104,1298,1492,1686,1880,2074}; 
int widthcenter=910;
float depthbound=0.75;
float areacoveragethresh=0.1;

/*for(int i=widthcenter; i!=widthcenter+u && ros::ok();i++){
      //iterate through width space
      for (int q=0;q!=(msg->height)-1 && ros::ok();q++){
        //iterate through top right quadrant
        int pixval=i+msg->width*q;
        //std::cout <<"pixel sel: "<<pixval<<'\n';
        //ROS_INFO("%g",msg->width);
        float depth=depths[pixval];
        //float depth=1;
        if (depth<depthbound){
            distance_count=distance_count+1;
            //std::cout <<"x: "<< q << " " << "y: " << i << " " <<"pixval: " << pixval << " " <<"distance: " << depth<< '\n';
        }
        
      }
       
    } */
    int height=(msg->height);
    int width=(msg->width);
   // float column_area=u*(msg->height);
   // float perc_covered=distance_count/column_area;
    float perc_covered=scandepth(widthcenter, u, width, height, depthbound, depths);
    if (perc_covered>areacoveragethresh){
      //Start scanning algorithm
      //Start left array first, then move to right 
      int leftacheived=0;
      //Scan Left 55degree FOV
      for (int ll=0; ll!=6; ll++){
          perc_covered=scandepth(leftwidtharr[ll], u, width, height, depthbound, depths);
          if (perc_covered<areacoveragethresh){
            leftacheived=1+leftacheived;
            std::cout <<"Left choice percCov:"<<perc_covered<<" column:"<<ll<<'\n';
            
            /* OLD_publishing
            std::stringstream ss;
            ss <<"Right percCov:" << std::to_string(perc_covered);
            command_msg.data = ss.str();
            commands.publish(command_msg);

            ss <<"Right percCov:" << std::to_string(perc_covered);
            msg_string.data = ss.str()
            commands.publish(msg_string);*/

            //to send floats, check data type of publisher:
            //msg_angle.data = 23.2;
            //commands.publish(msg_angle);

          /*if (nav_ready==true){
            std::stringstream ss;
            ss <<"Point:" << std::to_string(perc_covered);
            msg_string.data = ss.str();
            commands.publish(msg_string);
          }*/


          if (nav_ready==true){
            
            std::stringstream ss;
            ss <<"Point:value ";
            std::cout <<"Point:value"<<'\n';
            msg_string.data = ss.str();
            commands.publish(msg_string);  }



            break;
          }
      }
      //Did not get any passable columns in left try right
      if(leftacheived==0){
        for (int ll=0; ll!=6; ll++){
            perc_covered=scandepth(rightwidtharr[ll], u, width, height, depthbound, depths);
            if (perc_covered<areacoveragethresh){
              std::cout <<"Right percCov:"<<perc_covered<<" column:"<<ll<<'\n';
              


              break;
            }else{
              //Could not find left or right column free rotate
              std::cout <<"Rotate:"<<'\n';
            

            }  
        }
      }

    }else{
      std::cout <<"Center percCov:"<<perc_covered<<'\n';
    }
    

  
  }
  

  // Output the measure
  //ROS_INFO("Center distance : %g m", depths[centerIdx]);
  //ROS_INFO("test pixel : %g m", depths[2 + msg->width * 2]);


/**
 * Node main function
 */
int main(int argc, char** argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "zed_depth_subscriber");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called imageCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber subDepth = n.subscribe("/zed2/zed_node/depth/depth_registered", 10, depthCallback);

  ros::Subscriber subHandshake = n.subscribe("handshake", 10, handshakeCallback);

  ROS_INFO("zed depth started");
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */

  ros::Rate rate(1); //1Hz


  //create publisher object(created in global scope):
  commands = n.advertise<std_msgs::String>("sentry_commands", 1000);

  /*while(ros::ok()){
    command_msg.data = ss.str();
    commands.publish(command_msg);
    rate.sleep();
  }*/


  ros::spin();

  return 0;
}
