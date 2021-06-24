// Import libraries
#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <queue>

// Import header file
#include "moving_average_filter/filter_node.h"

// MAIN PROGRAM
int main(int argc, char **argv) {
  // Initialize Node
  ros::init(argc, argv, "filter_node");
  ros::NodeHandle n;

  // Initialize FilterNode object
  FilterNode node = FilterNode(&n);

  // Define Node loop frequency
  ros::Rate loop_rate(100);

  // Looping to subscribe message and publish filtered message
  ros::spin();

  return 0;
}

// ====================================================================================

// CONSTRUCTOR
FilterNode::FilterNode(ros::NodeHandle *n){
  // Initialize Node's subscriber object
  sub = n->subscribe("sensor_measurement", 25, &FilterNode::chatterCallback, this);
  
  // Initialize Node's publisher object
  pub = n->advertise<std_msgs::Float64>("sensor_measurement_filtered", 25);
  
  // Check whether parameter server has been defined
  if (ros::param::has("/size_N")) {
    ros::param::get("/size_N", N);
    ROS_INFO("Size N: [%d]\n", N);
  } else {
    ROS_INFO("PARAMETER '/size_N' HASN'T DEFINED");
    ROS_INFO("USING DEFAULT SIZE N = 5\n");
    N = 5;
  }
}

// METHODS
double FilterNode::filterData(double rawData) {
  // Update sum of values in the queue
  sumArr = sumArr - arr.front() + rawData;
  
  // Update elements in the queue
  arr.pop(); 
  arr.push(rawData);

  // Return filtered value
  return (sumArr / N);
}

void FilterNode::chatterCallback(const std_msgs::Float64::ConstPtr& msg) {  
  // Count the message
  ++count;

  // Print subscribed message
  ROS_INFO("Sensor measurement (raw data): [%lf]", msg->data);
  
  // Case where the data in queue are enough
  if (filtering) {
    datum = filterData(msg->data);
    
    // Print filtered data on the screen and publish it
    ROS_INFO("Filtered data: [%lf]" ,datum);
    publishMsg(datum);
  } 
  // Case where the data in queue aren't enough
  else {
    // Update sum of queue value and the queue itself
    sumArr += msg->data;
    arr.push(msg->data);

    // Check whether the data is enough
    if (count == N) {
      // Print filtered data on the screen and publish it
      ROS_INFO("Filtered data: [%lf]", (sumArr / N));
      publishMsg(sumArr / N);
      
      // Update to enough data in queue case
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