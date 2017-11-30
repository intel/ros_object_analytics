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

#include <ros/ros.h>
#include <ros/console.h>
#include "object_analytics_nodelet/const.h"
#include "object_analytics_nodelet/model/projector_impl.h"

namespace object_analytics_nodelet
{
namespace model
{
std::shared_ptr<ProjectorImpl> ProjectorImpl::instance_ = nullptr;

std::shared_ptr<ProjectorImpl> ProjectorImpl::instance()
{
  ROS_DEBUG("get projector instance");

  if (instance_ == nullptr)
  {
    ROS_INFO("create projector instance");
    instance_ = std::make_shared<ProjectorImpl>();
  }

  return instance_;
}

ProjectorImpl::ProjectorImpl()
{
  ros::Duration timeout(Const::kTimeoutCameraInfo);
  sensor_msgs::CameraInfoConstPtr info =
      ros::topic::waitForMessage<sensor_msgs::CameraInfo>(Const::kTopicDepthCameraInfo, timeout);

  if (!info)
  {
    ROS_FATAL_STREAM("timeout[" << Const::kTimeoutCameraInfo << "] when waiting for camerainfo");
    throw std::runtime_error("timeout for waiting camerainfo");
  }

  model_.fromCameraInfo(info);
}

void ProjectorImpl::project3dToPixel(const geometry_msgs::Point32& point, int& x, int& y)
{
  cv::Point3f xyz(point.x, point.y, point.z);
  cv::Point2f xy = model_.project3dToPixel(xyz);
  x = xy.x;
  y = xy.y;
}
}  // namespace model
}  // namespace object_analytics_nodelet
