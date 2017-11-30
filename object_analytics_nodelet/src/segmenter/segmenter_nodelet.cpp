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

#include <pluginlib/class_list_macros.h>
#include "object_analytics_nodelet/segmenter/segmenter_nodelet.h"

namespace object_analytics_nodelet
{
namespace segmenter
{
void SegmenterNodelet::onInit()
{
  ros::NodeHandle nh = getNodeHandle();

  try
  {
    impl_.reset(new Segmenter(nh));
  }
  catch (const std::runtime_error& e)
  {
    ROS_ERROR_STREAM("exception caught while starting segmenter nodelet, " << e.what());
    ros::shutdown();
  }
}
}  // namespace segmenter
}  // namespace object_analytics_nodelet
PLUGINLIB_EXPORT_CLASS(object_analytics_nodelet::segmenter::SegmenterNodelet, nodelet::Nodelet)
