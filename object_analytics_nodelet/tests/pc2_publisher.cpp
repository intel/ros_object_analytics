/*
 * Copyright (c) 2017 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#define PCL_NO_PRECOMPILE
#include <string>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include "tests/unittest_util.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "PointClodu2Publisher");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("pc2", 1);
  ros::Rate rate(1);

  sensor_msgs::PointCloud2 msg;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);

  readPointCloudFromPCD(std::string(RESOURCE_DIR) + "/cup.pcd", cloud);
  pcl::copyPointCloud(*cloud, *cloud2);

  pcl::toROSMsg(*cloud2, msg);
  while (ros::ok())
  {
    pub.publish(msg);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
