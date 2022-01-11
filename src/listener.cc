/*Assume gazebo running...
 *This node: 1) detect gazebo gui selection; 2) publish the name of the selected model.
 *References: http://gazebosim.org/tutorials?tut=topics_subscribed&cat=transport
 *            https://answers.ros.org/question/214814/subscribing-to-ros-and-gazebo-topic-in-one-node-ros-callback-failing/
 *Last modified on Dec 22,20 by Yubing Han
*/

#include <ros/ros.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include <iostream>
#include <string>
#include "std_msgs/String.h"

std::string modName;
int clicked=0;

/////////////////////////////////////////////////
// Function is called everytime a message is received.
void cb(ConstSelectionPtr &_msg)
{
  
  std::stringstream ss(_msg->DebugString());
  std::istream_iterator<std::string> begin(ss);
  std::istream_iterator<std::string> end;
  std::vector<std::string> vstrings(begin, end);
  //std::cout<<_msg->DebugString();
  if (vstrings[5]=="true"){
    modName=vstrings[3];
    clicked=1;
  }
}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  // Load gazebo & ros
  gazebo::client::setup(_argc, _argv);
  ros::init(_argc,_argv,"listener");

  // Create gazebo node
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Create ros node
  ros::NodeHandle n;
  ros::Publisher mod_name_pub=n.advertise<std_msgs::String>("mod_name",1000);

  // Listen to Gazebo selection topic
  gazebo::transport::SubscriberPtr sub = node->Subscribe("~/selection", cb);


  while (ros::ok()){
    gazebo::common::Time::MSleep(10);
    if (clicked==1){
      std_msgs::String msg;
      msg.data=modName;
      mod_name_pub.publish(msg);
      std::cout<< modName << " selected..............\n";
      clicked=0;
    }
    ros::spinOnce;
  }

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}
