#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <vector>


class LaserScanFilter {
  public:
    LaserScanFilter() {
      ros::NodeHandle nh;
      scan_sub_ = nh.subscribe("scan", 1, &LaserScanFilter::scanCallback, this);
      scan_filtered_pub_ = nh.advertise<sensor_msgs::LaserScan>("scan_filtered",1,false);
  
      ros::NodeHandle pnh("~");
      XmlRpc::XmlRpcValue my_list;
      pnh.getParam("filter_index_list", my_list);
      if (my_list.getType() != XmlRpc::XmlRpcValue::TypeArray) ros::shutdown();
  
      for (int32_t i = 0; i < my_list.size(); ++i) {
        if (my_list[i].getType() != XmlRpc::XmlRpcValue::TypeArray) 
          ros::shutdown();
        int min = my_list[i][0];
        int max = my_list[i][1];
        addFilterIndices(min, max);
        ROS_INFO("scan filter index interval %d : min: %d max: %d",i, min, max);
      }
    }
  
    void scanCallback(const sensor_msgs::LaserScan& scan) {
      this->pubFilteredScan(scan);
    }
  
    void pubFilteredScan(const sensor_msgs::LaserScan& scan) {
      filtered_scan_ = scan;
      size_t filter_indices_size = filter_indices_.size();
      for (size_t i = 0; i < filter_indices_size; ++i) {
        filtered_scan_.ranges[filter_indices_[i]] = scan.range_max + 1.0;
      }
      scan_filtered_pub_.publish(filtered_scan_);
    }
  
    void addFilterIndices(size_t min, size_t max) {
      for (size_t i = min; i < max; ++i) {
        filter_indices_.push_back(i);
      }
    }
  
  protected:
    ros::Subscriber scan_sub_;
    ros::Publisher scan_filtered_pub_;
    sensor_msgs::LaserScan filtered_scan_;
    std::vector<size_t> filter_indices_;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "hector_turtlebot_scan_filter");
  LaserScanFilter lsf;
  ros::spin();
  return 0;
}
