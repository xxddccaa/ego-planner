#include "simple_map.hpp"
#include <iostream>
#include <cmath>

using namespace mockamap2;

SimpleMap::SimpleMap() {
  // 构造函数
}

SimpleMap::BasicInfo SimpleMap::getInfo() const {
  return info;
}

void SimpleMap::setInfo(const BasicInfo &value) {
  info = value;
}

void SimpleMap::generateFourPillars() {
  // 清空点云
  info.cloud->points.clear();
  
  // 获取参数
  double pillar_width = 0.5;  // 柱子宽度（米）
  double pillar_height = 2.0; // 柱子高度（米）
  
  // 从参数服务器读取柱子参数
  info.nh_private->param("pillar_width", pillar_width, 0.5);
  info.nh_private->param("pillar_height", pillar_height, 2.0);
  
  // 计算四个柱子的位置（在四个象限）
  double map_half_x = info.map_size_x / 2.0;
  double map_half_y = info.map_size_y / 2.0;
  
  // 四个柱子的中心位置
  std::vector<std::pair<double, double>> pillar_positions = {
    {map_half_x * 0.6, map_half_y * 0.6},   // 第一象限
    {-map_half_x * 0.6, map_half_y * 0.6},  // 第二象限
    {-map_half_x * 0.6, -map_half_y * 0.6}, // 第三象限
    {map_half_x * 0.6, -map_half_y * 0.6}   // 第四象限
  };
  
  // 生成四个柱子
  for (const auto& pos : pillar_positions) {
    addPillar(pos.first, pos.second, pillar_width, pillar_height);
  }
  
  // 设置点云属性
  info.cloud->width = info.cloud->points.size();
  info.cloud->height = 1;
  info.cloud->is_dense = true;
  
  // 转换为ROS消息
  pcl2ros();
  
  ROS_INFO("Generated four pillars map with %d points", (int)info.cloud->points.size());
}

void SimpleMap::addPillar(double center_x, double center_y, double width, double height) {
  double half_width = width / 2.0;
  
  // 计算柱子边界
  double x_min = center_x - half_width;
  double x_max = center_x + half_width;
  double y_min = center_y - half_width;
  double y_max = center_y + half_width;
  
  // 生成柱子的点云
  for (double x = x_min; x <= x_max; x += info.resolution) {
    for (double y = y_min; y <= y_max; y += info.resolution) {
      for (double z = 0; z <= height; z += info.resolution) {
        pcl::PointXYZ point;
        point.x = x;
        point.y = y;
        point.z = z;
        info.cloud->points.push_back(point);
      }
    }
  }
}

void SimpleMap::pcl2ros() {
  pcl::toROSMsg(*info.cloud, *info.output);
  info.output->header.frame_id = "world";
} 