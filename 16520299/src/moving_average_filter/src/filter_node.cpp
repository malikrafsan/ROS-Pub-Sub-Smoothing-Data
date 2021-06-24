#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <exception>
//#include <vector>
#include <queue>
#include <sstream>


class FilterNode{
  private:
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
    FilterNode(ros::NodeHandle *n){
      sub = n->subscribe("sensor_measurement", 25, &FilterNode::chatterCallback, this);
      pub = n->advertise<std_msgs::Float64>("sensor_measurement_filtered", 25);
      if (ros::param::has("/size_N")) {
        ros::param::get("/size_N", N);
      } else {
        ROS_INFO("PARAMETER '/size_N' HAS TO BE DEFINE FIRST");
        throw std::exception();
      }
      ROS_INFO("PARAM: [%d]", N);
      //isiArr();
    }

    /*
    void isiArr() {
      for (int i=0;i<N;i++){
        arr[i] = 0;
      }
    }
    */

    double filterData(double rawData) {
      sumArr = sumArr - arr.front() + rawData;
      //arr[count % N] = rawData;
      arr.pop(); 
      arr.push(rawData);
      return (sumArr / N);
    }

    void chatterCallback(const std_msgs::Float64::ConstPtr& msg) {
      ++count;
      ROS_INFO("I heard: [%lf]", msg->data);
      if (filtering) {
        datum = filterData(msg->data);
        ROS_INFO("Filtered: [%g]" ,datum);
      } else {
        sumArr += msg->data;
        arr.push(msg->data);
        if (count == N) {
          ROS_INFO("Filtered: [%g]", (sumArr / N));
          filtering = true;
        } else {
          ROS_INFO("Still waiting enough data");
        }
      }
      ROS_INFO("Count: %d\n", count);
      showq(arr);
      
      sensor_measurement_filtered.data = datum;
      pub.publish(sensor_measurement_filtered);
    }

    void showq(std::queue<double> gq) {
        std::queue<double> g = gq;
        while (!g.empty()) {
            //cout << '\t' << g.front();
            ROS_INFO("%g \t",g.front());
            g.pop();
        }
        ROS_INFO("\n");
    }

};

int main(int argc, char **argv) {
  ros::init(argc, argv, "filter_node");

  ros::NodeHandle n;

  FilterNode node = FilterNode(&n);

  ros::Rate loop_rate(0.2);
  while (ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
  }

  return 0;
}