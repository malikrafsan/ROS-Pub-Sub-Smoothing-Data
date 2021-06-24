#ifndef FILTER_NODE_H
#define FILTER_NODE_H
#include <ros/ros.h>

void sayHello();

class FilterNode{
  private:
    // ATTRIBUTE
    int N;
    int count = 0;
    double datum;
    double sumArr = 0;
    bool filtering = false;
    std::queue<double> arr;
    std_msgs::Float64 sensor_measurement_filtered;
    ros::Publisher pub;
    ros::Subscriber sub; 

  public:
    // CONSTRUCTOR
    FilterNode(ros::NodeHandle *n);

    // METHODS
    double filterData(double rawData);
    void chatterCallback(const std_msgs::Float64::ConstPtr& msg);
    void publishMsg(double datum);

};
#endif