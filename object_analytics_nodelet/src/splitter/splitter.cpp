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

#include <pcl_conversions/pcl_conversions.h>

#include "object_analytics_nodelet/model/object3d.h"
#include "object_analytics_nodelet/splitter/splitter.h"

namespace object_analytics_nodelet
{
namespace splitter
{
using object_analytics_nodelet::model::PointT;

void Splitter::split(const sensor_msgs::PointCloud2::ConstPtr& points, sensor_msgs::Image::Ptr& image)
{
  try
  {
    std_msgs::Header header = points->header;
    pcl::toROSMsg(*points, *image);
    image->header = header;
  }
  catch (const std::runtime_error& e)
  {
    ROS_ERROR_STREAM("caught exception " << e.what() << " while splitting, skip this message");
    return;
  }
}
}  // namespace splitter
}  // namespace object_analytics_nodelet
