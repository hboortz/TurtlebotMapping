#include <vector>
#include <iostream>

#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>

class HectorNode {
	public:
		HectorNode();
		void scanCallback(const sensor_msgs::LaserScan &scan);
};

HectorNode::HectorNode(){}

void HectorNode::scanCallback(const sensor_msgs::LaserScan &scan) {
		// std::cout << "HERE!!!!";
		// std::cout << scan;
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "hector_node");
	HectorNode maps = HectorNode();
  ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("scan", 1000, &HectorNode::scanCallback, &maps);
  ros::spin();
  return 0;
}
