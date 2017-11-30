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

#include <vector>
#include <std_msgs/Header.h>
#include <cv_bridge/cv_bridge.h>
#include "object_analytics_nodelet/tracker/tracking_nodelet.h"
#include "object_analytics_nodelet/const.h"

PLUGINLIB_EXPORT_CLASS(object_analytics_nodelet::tracker::TrackingNodelet, nodelet::Nodelet);

namespace object_analytics_nodelet
{
namespace tracker
{
// TrackingNodelet class implementation
using SubscribeImg = message_filters::Subscriber<sensor_msgs::Image>;
using SubscribeObjs = message_filters::Subscriber<object_msgs::ObjectsInBoxes>;
using Synchronizer = message_filters::TimeSynchronizer<object_msgs::ObjectsInBoxes, sensor_msgs::Image>;

void TrackingNodelet::onInit()
{
  ros::NodeHandle nh = getNodeHandle();
  sub_rgb_ = nh.subscribe(Const::kTopicRgb, 6, &TrackingNodelet::rgb_cb, this);
  sub_obj_ = nh.subscribe(Const::kTopicDetection, 6, &TrackingNodelet::obj_cb, this);
  pub_tracking_ = nh.advertise<object_analytics_msgs::TrackedObjects>(Const::kTopicTracking, 10);
  tm_ = std::make_shared<TrackingManager>();
  last_detection_ = ros::Time();
  this_detection_ = ros::Time();
  last_obj_ = nullptr;
  this_obj_ = nullptr;
}

TrackingNodelet::~TrackingNodelet()
{
}

void TrackingNodelet::rgb_cb(const sensor_msgs::ImageConstPtr& img)
{
  rgbs_.push_back(img);
}

void TrackingNodelet::obj_cb(const object_msgs::ObjectsInBoxesConstPtr& objs)
{
  last_detection_ = this_detection_;
  this_detection_ = objs->header.stamp;
  last_obj_ = this_obj_;
  this_obj_ = objs;
  if (last_detection_ == ros::Time())
  {
    return;
  }

  std::vector<sensor_msgs::ImageConstPtr>::iterator rgb = rgbs_.begin();
  while (rgb != rgbs_.end())
  {
    if ((*rgb)->header.stamp < last_detection_)
    {
      ROS_DEBUG("slower, dropped");
      rgb = rgbs_.erase(rgb);
      continue;
    }
    if ((*rgb)->header.stamp >= this_detection_)
    {
      ROS_DEBUG("faster, break");
      break;
    }
    cv::Mat mat_cv = cv_bridge::toCvShare(*rgb, "bgr8")->image;
    if ((*rgb)->header.stamp == last_detection_)
    {
      ROS_DEBUG("rectify!");
      tm_->detect(mat_cv, last_obj_);
    }
    else
    {
      ROS_DEBUG("track!");
      tm_->track(mat_cv);
    }
    tracking_publish((*rgb)->header);
    rgb = rgbs_.erase(rgb);
  }
}

void TrackingNodelet::tracking_publish(const std_msgs::Header& header)
{
  boost::shared_ptr<object_analytics_msgs::TrackedObjects> msg =
      boost::make_shared<object_analytics_msgs::TrackedObjects>();
  msg->header = header;
  tm_->getTrackedObjs(msg);
  pub_tracking_.publish(msg);
}

}  // namespace tracker
}  // namespace object_analytics_nodelet
