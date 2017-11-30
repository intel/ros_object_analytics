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
#include <string>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Image.h>

#include "object_analytics_nodelet/const.h"
#include "object_analytics_nodelet/model/projector_impl.h"
#include "object_analytics_nodelet/splitter/splitter.h"

namespace object_analytics_nodelet
{
namespace splitter
{
using object_analytics_nodelet::model::ProjectorImpl;

Splitter::Splitter(ros::NodeHandle& nh)
{
  projector_ = ProjectorImpl::instance();
  sub_pc2_ = nh.subscribe(Const::kTopicRegisteredPC2, 1, &Splitter::cbSplit, this);
  pub_2d_ = nh.advertise<sensor_msgs::Image>(Const::kTopicRgb, 1);
  pub_3d_ = nh.advertise<sensor_msgs::PointCloud2>(Const::kTopicPC2, 1);
}

void Splitter::cbSplit(const sensor_msgs::PointCloud2::ConstPtr& points)
{
  try
  {
    sensor_msgs::ImagePtr image(new sensor_msgs::Image());
    pcl::toROSMsg(*points, *image);
    image->header = points->header;
    pub_2d_.publish(image);
    pub_3d_.publish(points);
  }
  catch (const std::runtime_error& e)
  {
    ROS_ERROR_STREAM("caught exception " << e.what() << " while splitting, skip this message");
    return;
  }
}
}  // namespace splitter
}  // namespace object_analytics_nodelet
