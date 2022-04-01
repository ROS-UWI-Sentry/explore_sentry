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


int global_run_once = 0;

/**
 * Subscriber callback
 */

void depthCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  std::stringstream ss;
  // Get a pointer to the depth values casting the data
  // pointer to floating point
  float* depths = (float*)(&msg->data[0]);

  // Image coordinates of the center pixel
  int u = msg->width / 2;
  int v = msg->height / 2;

  // Linear index of the center pixel
  int centerIdx = u + msg->width * v;
  int alpha=4; //width aspect must be greater than 2
  int beta=3; //height aspect must be greater than 2
  float distance_count=0;
  float point_count=0;
//Right half plane
    for(int i=v; i!=v+(msg->width)/alpha && ros::ok();i++){
      //iterate through width space
      for (int q=u;q!=u+(msg->height)/beta && ros::ok();q++){
        //iterate through top right quadrant
        int pixval=q+msg->width*i;
        //ROS_INFO("%g",msg->width);
        float depth=depths[pixval];
        if (depth<0.7){
            distance_count=distance_count+1;
            //std::cout <<"x: "<< q << " " << "y: " << i << " " <<"pixval: " << pixval << " " <<"distance: " << depth<< '\n';
        }
        
      }
      for (int q=u-(msg->height)/beta; q!=u  && ros::ok(); q++){
        //iterate through bottom right quadrant
        int pixval=q+msg->width*i;
        float depth=depths[pixval];
        if (depth<0.7){
            distance_count=distance_count+1;
        }
       
      } 
      
    }
//Left half plane
    for(int i=v-(msg->width)/alpha; i!=v && ros::ok();i++){
      //iterate through width space
      for (int q=u;q!=u+(msg->height)/beta && ros::ok();q++){
        //iterate through top left quadrant
        int pixval=q+msg->width*i;
        //ROS_INFO("%g",msg->width);
        float depth=depths[pixval];
        if (depth<0.7){
            distance_count=distance_count+1;
            //std::cout <<"x: "<< q << " " << "y: " << i << " " <<"pixval: " << pixval << " " <<"distance: " << depth<< '\n';
        }
        //ROS_INFO("P: %g m",pixval);
        //ROS_INFO("z: %g m",depths[pixval]);
      }
      for (int q=u-(msg->height)/beta; q!=u  && ros::ok(); q++){
        //iterate through bottom left quadrant
        int pixval=q+msg->width*i;
        float depth=depths[pixval];
        if (depth<0.7){
            distance_count=distance_count+1;
            //std::cout <<"x: "<< q << " " << "y: " << i << " " <<"pixval: " << pixval << " " <<"distance: " << depth<< '\n';
        }
      } 

    }
    float pixel_size=4*((msg->height)/beta)*((msg->width)/alpha);
    float perc_covered=distance_count/pixel_size;
    std::cout <<"RESET: " << distance_count<<" Cov:"<<perc_covered<<'\n';

    distance_count=0;
  

  
  // Output the measure
  //ROS_INFO("Center distance : %g m", depths[centerIdx]);
  //ROS_INFO("test pixel : %g m", depths[2 + msg->width * 2]);
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
  ros::init(argc, argv, "zed_video_subscriber");

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
  ROS_INFO("zed depth started");
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
