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
#include <string>  
#include <twist_to_motor_rps/Num.h>
#include <math.h>

int global_run_once = 0;
std_msgs::Float64 msg_angle;
ros::Publisher pub_commands;
ros::Publisher pub_depth_ready;
ros::Publisher pub_to_state_machine;
std_msgs::String msg_string;
std_msgs::Float64 message_float;
std::stringstream msg_string_string_stream;

std_msgs::Bool depth_ready_bool;

twist_to_motor_rps::Num state_trajectory;

//std_msgs::Bool handshake_from_nav;
bool handshake_from_nav;

int test_count = 0;


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

void handshake_from_nav_callback(const std_msgs::Bool::ConstPtr& msg)
{
  handshake_from_nav=msg->data;
  if (handshake_from_nav==true){
    //send message here(save variable in code lower down)
    pub_commands.publish(msg_string);
    //ready to receive data
    depth_ready_bool.data=true;
    pub_depth_ready.publish(depth_ready_bool);
  }
   
  std::cout <<"inside of callback for handshake"<<handshake_from_nav<<'\n';
}
  
void data_from_state_machine(const twist_to_motor_rps::Num::ConstPtr& msg){
  std::cout <<"data from explore_sentry:"<< msg->num[0] <<'\n';
  
  //this is python code
 // phidk=commands_from_depth_data+phik
  //xd=xk+1*cos(phidkp)
 // yd=yk+1*sin(phidkp)
  float xk=msg->num[0];
  float yk=msg->num[1];
  float phik=msg->num[2];

  //my attempt at convering from python to c++
  float phidk=message_float.data+phik;
  float xd=xk+cos(phidk);
  float yd=yk+sin(phidk);

  //pub_commands.publish(message_float);
  // send xd yd and phid in that order
  state_trajectory.num={xd,yd,phidk};
  pub_to_state_machine.publish(state_trajectory);
  
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
  int cammid=1104; //zed camera middle
  float mmpx=1.29;
//int leftwidtharr[6]={164,358,552,746,910,940};
  int leftwidtharr[6]={910,716,522,328,134}; //
  int rightwidtharr[6]={1104,1298,1492,1686,1880,2074}; 
  int widthcenter=910;
  float depthbound=0.75;
  float areacoveragethresh=0.1;

    int height=(msg->height);
    int width=(msg->width);
   // float column_area=u*(msg->height);
   // float perc_covered=distance_count/column_area;
    float perc_covered=scandepth(widthcenter, u, width, height, depthbound, depths);
   // std::string msg_string="";
    if (perc_covered>areacoveragethresh){
      //Start scanning algorithm
      //Start left array first, then move to right 
      int acheived=0;
      //Scan Left 55degree FOV
      for (int ll=0; ll!=6; ll++){
          perc_covered=scandepth(leftwidtharr[ll], u, width, height, depthbound, depths);
          if (perc_covered<areacoveragethresh){
            acheived=1+acheived;
            float distcent=tan(0.001*mmpx*(cammid-(leftwidtharr[ll]+u)/2));
            //msg_string_string_stream << "L:" << distcent;
            message_float.data=distcent;
            std::cout <<"Left choice percCov:"<<perc_covered<<" column:"<<ll<<" ang: "<<distcent<<'\n';
            break;
          }
          perc_covered=scandepth(rightwidtharr[ll], u, width, height, depthbound, depths);
            if (perc_covered<areacoveragethresh){
              acheived=1+acheived;
              std::cout <<"Right percCov:"<<perc_covered<<" column:"<<ll<<'\n';
              float distcent=tan(0.001*mmpx*(-cammid+(rightwidtharr[ll]+u)/2));
              message_float.data=distcent;
              break;
            }

      }

      if(acheived==0){
            std::cout <<"Rotate:"<<'\n';
            float distcent=3.14;
            message_float.data=distcent;

      }
      //Did not get any passable columns in left try right

    }else{
      std::cout <<"Center percCov:"<<perc_covered<<'\n';
      message_float.data=0;
    }

  }
  


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
  ros::init(argc, argv, "nav_sensor");

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

  //ros::Subscriber subHandshake = n.subscribe("handshake_from_nav", 10, handshake_from_nav_callback);

  ros::Subscriber subStateMachineData = n.subscribe("state_machine_to_nav_sensor", 10 , data_from_state_machine);

  ROS_INFO("zed depth started");
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */

  ros::Rate rate(1); //1Hz


  //create publisher object(created in global scope):
  //pub_commands = n.advertise<std_msgs::String>("commands_from_depth", 1000);
  pub_commands = n.advertise<std_msgs::Float64>("commands_from_depth", 1000);
  pub_depth_ready = n.advertise<std_msgs::Bool>("handshake_from_depth", 1000, true);
  pub_to_state_machine= n.advertise<twist_to_motor_rps::Num>("nav_sensor_data", 1000, true);

  //to allow node to register with master etc
  rate.sleep();

  //to initialize the handshake as false
   depth_ready_bool.data=false;
   pub_depth_ready.publish(depth_ready_bool);
   std::cout <<"test string"<<'\n';

  /*while(ros::ok()){
    command_msg.data = ss.str();
    pub_commands.publish(command_msg);
    rate.sleep();
  }*/


  ros::spin();

  return 0;
}
