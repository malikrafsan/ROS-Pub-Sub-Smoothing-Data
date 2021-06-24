#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <typeinfo>

int count = 0;
double datum;
std_msgs::Float64 sensor_measurement_filtered;

void chatterCallback(const std_msgs::Float64::ConstPtr& msg) {
  ROS_INFO("I heard: [%f]", msg->data);
  datum = (msg->data * 10) * (msg->data * 10);
  ROS_INFO("added: [%f]" ,datum);
  ROS_INFO("count: %d", count);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "filter_node");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("sensor_measurement", 25, chatterCallback);
  ros::Publisher pub = n.advertise<std_msgs::Float64>("sensor_measurement_filtered", 25);
  
  //std::cout << "Main program" << datum << std::endl;

  //ros::spin();

	ros::Rate loop_rate(100);

	ros::spinOnce();

  while (ros::ok()){

    sensor_measurement_filtered.data = datum;

    pub.publish(sensor_measurement_filtered);

		ros::spinOnce();
		loop_rate.sleep();
		++count;
  }


  return 0;
}