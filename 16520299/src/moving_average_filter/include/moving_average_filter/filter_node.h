#ifndef FILTER_NODE_H
#define FILTER_NODE_H
#include <ros/ros.h>

class FilterNode{
  private:
    // ATTRIBUTE
    int N;                                              // Size of data which are averaged
    int count = 0;                                      // Messages counter
    double datum;                                       // Filtered data
    double sumArr = 0;                                  // Sum of data value in the queue
    bool filtering = false;                             // Boolean flag for switching case to start filtering data
    std::queue<double> arr;                             // Queue data structure
    std_msgs::Float64 sensor_measurement_filtered;      // Message that has been filtered
    ros::Publisher pub;                                 // Node's publisher object
    ros::Subscriber sub;                                // Node's subscriber object

  public:
    // CONSTRUCTOR
    FilterNode(ros::NodeHandle *n);
    // Construct FilterNode object

    // METHODS
    double filterData(double rawData);
    // Return value of the message that has been filtered using moving average

    void chatterCallback(const std_msgs::Float64::ConstPtr& msg);
    // I.S. Subscribed message has been catched
    // F.S. Filtered message has been published and informations 
    // has been printed on the screen

    void publishMsg(double datum);
    // I.S. Filtered data is defined
    // F.S. Filtered data has been published

};
#endif