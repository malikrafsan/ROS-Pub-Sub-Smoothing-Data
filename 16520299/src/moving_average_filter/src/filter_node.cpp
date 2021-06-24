#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <typeinfo>

class FilterNode{
  private:
    int N;
    int count = 0;
    double datum;
    double arr[5];
    double sumArr = 0;
    std_msgs::Float64 sensor_measurement_filtered;

    ros::Publisher pub;
    ros::Subscriber sub; 

  public:
    FilterNode(ros::NodeHandle *n){
      sub = n->subscribe("sensor_measurement", 25, &FilterNode::chatterCallback, this);
      pub = n->advertise<std_msgs::Float64>("sensor_measurement_filtered", 25);
      ros::param::get("/size_array", N);
      ROS_INFO("PARAM: [%d]", N);
      isiArr();
    }

    void isiArr() {
      for (int i=0;i<N;i++){
        arr[i] = 0;
      }
    }

    double filterData(double rawData) {
      sumArr = sumArr - arr[count % N] + rawData;
      arr[count % N] = rawData;
      return (sumArr / N);
    }

    void chatterCallback(const std_msgs::Float64::ConstPtr& msg) {
      ROS_INFO("I heard: [%f]", msg->data);
      datum = filterData(msg->data);
      ROS_INFO("added: [%f]" ,datum);
      ROS_INFO("count: %d", count);
      
      sensor_measurement_filtered.data = datum;
      pub.publish(sensor_measurement_filtered);
      
      ++count;
    }

};

//const int N = 5;
//int count = 0;
//double datum;
//double arr[N];
//double sumArr = 0;
//std_msgs::Float64 sensor_measurement_filtered;

//void isiArr() {
//  for (int i=0;i<N;i++){
//    arr[i] = 0;
//  }
//}

/*
double filterData(double rawData) {
  sumArr = sumArr - arr[count % N] + rawData;
  arr[count % N] = rawData;
  return (sumArr / N);
}

void chatterCallback(const std_msgs::Float64::ConstPtr& msg) {
  ROS_INFO("I heard: [%f]", msg->data);
  datum = filterData(msg->data);
  ROS_INFO("added: [%f]" ,datum);
  ROS_INFO("count: %d", count);
  ++count;
}
*/

int main(int argc, char **argv) {
  ros::init(argc, argv, "filter_node");

  ros::NodeHandle n;

  FilterNode node = FilterNode(&n);

  /*
  ros::Subscriber sub = n.subscribe("sensor_measurement", 25, chatterCallback);
  ros::Publisher pub = n.advertise<std_msgs::Float64>("sensor_measurement_filtered", 25);

	ros::Rate loop_rate(0.2);

  isiArr();

  */

  ros::Rate loop_rate(0.2);
  while (ros::ok()){

    //sensor_measurement_filtered.data = datum;

    //pub.publish(sensor_measurement_filtered);

		ros::spinOnce();
		loop_rate.sleep();
  }


  return 0;
}