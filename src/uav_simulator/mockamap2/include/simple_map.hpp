#ifndef SIMPLE_MAP_HPP
#define SIMPLE_MAP_HPP

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

namespace mockamap2 {

class SimpleMap {
public:
  typedef struct BasicInfo {
    ros::NodeHandle *nh_private;
    double resolution;
    double map_size_x;
    double map_size_y;
    double map_size_z;
    sensor_msgs::PointCloud2 *output;
    pcl::PointCloud<pcl::PointXYZ> *cloud;
  } BasicInfo;

  BasicInfo getInfo() const;
  void setInfo(const BasicInfo &value);

public:
  SimpleMap();

public:
  void generateFourPillars();

private:
  BasicInfo info;

private:
  void pcl2ros();
  void addPillar(double center_x, double center_y, double width, double height);
};

} // namespace mockamap2

#endif // SIMPLE_MAP_HPP 