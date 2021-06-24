#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <exception>
#include <queue>
#include "moving_average_filter/filter_node.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "filter_node");

  ros::NodeHandle n;

  FilterNode node = FilterNode(&n);

  ros::Rate loop_rate(0.2);
  ros::spin();

  return 0;
}

// CONSTRUCTOR
FilterNode::FilterNode(ros::NodeHandle *n){
  sub = n->subscribe("sensor_measurement", 25, &FilterNode::chatterCallback, this);
  pub = n->advertise<std_msgs::Float64>("sensor_measurement_filtered", 25);
  if (ros::param::has("/size_N")) {
    ros::param::get("/size_N", N);
  } else {
    ROS_INFO("PARAMETER '/size_N' HAS TO BE DEFINE FIRST");
    throw std::exception();
  }
  ROS_INFO("Size N: [%d]\n", N);
}

// METHODS
double FilterNode::filterData(double rawData) {
  sumArr = sumArr - arr.front() + rawData;
  arr.pop(); 
  arr.push(rawData);
  return (sumArr / N);
}

void FilterNode::chatterCallback(const std_msgs::Float64::ConstPtr& msg) {
  ++count;
  ROS_INFO("I heard: [%lf]", msg->data);
  if (filtering) {
    datum = filterData(msg->data);
    ROS_INFO("Filtered: [%lf]" ,datum);
    publishMsg(datum);
  } else {
    sumArr += msg->data;
    arr.push(msg->data);
    if (count == N) {
      ROS_INFO("Filtered: [%lf]", (sumArr / N));
      publishMsg(sumArr / N);
      filtering = true;
    } else {
      ROS_INFO("Still waiting enough data");
    }
  }
  ROS_INFO("Count: %d\n", count);
}

void FilterNode::publishMsg(double datum) {
  sensor_measurement_filtered.data = datum;
  pub.publish(sensor_measurement_filtered);
}