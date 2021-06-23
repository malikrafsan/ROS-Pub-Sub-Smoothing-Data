#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <typeinfo>


void chatterCallback(const std_msgs::Float64::ConstPtr& msg) {
  ROS_INFO("I heard: [%f]", msg->data);
  double datum = msg->data + 2;
  ROS_INFO("added: [%f]" ,datum);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "filter_node");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("sensor_measurement", 25, chatterCallback);
  
  ros::spin();

  return 0;
}