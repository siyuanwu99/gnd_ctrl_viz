/**
 * @file
 * @brief This node reads the registered point cloud and publishes the cloud map in the world frame.
 */

#include <ikd-Tree/ikd_Tree.h>
#include <math.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>
#include <iostream>
#include <string>

/** DEBUG */
#define BACKWARD_HAS_DW 1
#include "backward.hpp"
namespace backward {
backward::SignalHandling sh;
}

/** parameters */
double map_res_(0.2);
double map_ground_height_(-0.1);
double map_ceil_height_(3.2);
int    map_freq_(10);

std::string map_frame_;

/** global variables */
bool           is_triggered_(false);
ros::Publisher pub_map_;

pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_map_;

using PointType   = pcl::PointXYZI;
using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

std::unique_ptr<KD_TREE<PointType>> ikdtree_;

/**
 * @brief recieve the registered point cloud and build the cloud map
 *
 * @param msg the registered point cloud in the world frame
 */
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  ros::Time time_start = ros::Time::now();

  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::fromROSMsg(*msg, cloud);
  int num = cloud.points.size();
  if (!is_triggered_) {
    ikdtree_->Build(cloud.points);
    is_triggered_ = true;
    return;
  }

  /** Add points to ikdtree */
  PointVector cloud_increment;
  cloud_increment.reserve(num);
  for (int i = 0; i < num; ++i) {
    if (cloud.points[i].z < map_ground_height_ || cloud.points[i].z > map_ceil_height_) {
      continue;
    }
    cloud_increment.push_back(cloud.points[i]);
  }

  ikdtree_->Add_Points(cloud_increment, true);

  ROS_INFO("[lidar_mapping] cloud_map size: %d", ikdtree_->validnum());
  ROS_INFO("[lidar_mapping] cloud_map takes %f ms", (ros::Time::now() - time_start).toSec() * 1000);
}

/**
 * @brief publish the cloud map in a fixed frequency
 *
 * @param event timer event
 */
void timerCallback(const ros::TimerEvent& event) {
  if (!is_triggered_) {
    return;
  }
  // publish cloud_map

  // cloud_map_->header.frame_id = map_frame_;
  // cloud_map_->header.stamp    = ros::Time::now().toSec();

  PointVector().swap(ikdtree_->PCL_Storage);
  ikdtree_->flatten(ikdtree_->Root_Node, ikdtree_->PCL_Storage, NOT_RECORD);

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
  cloud->points = ikdtree_->PCL_Storage;

  cloud->header.frame_id = map_frame_;
  cloud->header.stamp    = ros::Time::now().toSec();
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*cloud, msg);
  pub_map_.publish(msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "lidar_mapping");
  ros::NodeHandle nh;

  nh.param<double>("map_resolution", map_res_, 0.2);
  nh.param<double>("map_ground_height", map_ground_height_, -0.1);
  nh.param<double>("map_ceil_height", map_ceil_height_, 3.2);

  nh.param<std::string>("map_frame", map_frame_, "world");
  nh.param<int>("map_frequency", map_freq_, 10);

  ROS_INFO_ONCE("[lidar_mapping] map_resolution: %f \n",
                "[lidar_mapping] map_frequency: %d \n"
                "[lidar_mapping] map_frame: %s \n",
                map_res_, map_freq_, map_frame_.c_str());

  // cloud_map_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  // cloud_map_->points.clear();
  // cloud_map_->width  = 0;
  // cloud_map_->height = 1;

  ikdtree_ = std::make_unique<KD_TREE<PointType>>();
  ikdtree_->set_downsample_param(map_res_);

  ros::Subscriber sub_pcl = nh.subscribe("/cloud_registered", 1, cloudCallback);

  pub_map_ = nh.advertise<sensor_msgs::PointCloud2>("/cloud_map", 1);

  ros::Timer timer = nh.createTimer(ros::Duration(1.0 / map_freq_), timerCallback);

  ros::spin();
}
