#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "simple_map.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "mockamap2");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // 创建发布器
  ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("mock_map", 1);
  
  // 创建点云对象
  pcl::PointCloud<pcl::PointXYZ> cloud;
  sensor_msgs::PointCloud2 output;

  // 获取参数
  double resolution;
  double map_size_x, map_size_y, map_size_z;
  double update_freq;

  nh_private.param("resolution", resolution, 0.1);
  nh_private.param("map_size_x", map_size_x, 10.0);
  nh_private.param("map_size_y", map_size_y, 10.0);
  nh_private.param("map_size_z", map_size_z, 3.0);
  nh_private.param("update_freq", update_freq, 1.0);

  // 创建地图生成器
  mockamap2::SimpleMap map;
  
  // 设置地图信息
  mockamap2::SimpleMap::BasicInfo info;
  info.nh_private = &nh_private;
  info.resolution = resolution;
  info.map_size_x = map_size_x;
  info.map_size_y = map_size_y;
  info.map_size_z = map_size_z;
  info.output = &output;
  info.cloud = &cloud;
  
  map.setInfo(info);

  // 生成四根柱子的地图
  map.generateFourPillars();

  ROS_INFO("Mockamap2 initialized with four pillars");
  ROS_INFO("Map size: %.1f x %.1f x %.1f meters", map_size_x, map_size_y, map_size_z);
  ROS_INFO("Resolution: %.2f meters", resolution);

  // 发布循环
  ros::Rate loop_rate(update_freq);
  while (ros::ok()) {
    pcl_pub.publish(output);
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
} 