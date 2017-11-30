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
#include <cv_bridge/cv_bridge.h>
#include "object_analytics_nodelet/tracker/tracking.h"

namespace object_analytics_nodelet
{
namespace tracker
{
const int32_t Tracking::kAgeingThreshold = 16;

Tracking::Tracking(int32_t tracking_id, const std::string& name, const cv::Rect2d& rect)
{
  tracking_id_ = tracking_id;
  obj_name_ = name;
  rect_ = rect;
  detected_ = false;
}

Tracking::~Tracking()
{
  if (!tracker_.empty())
  {
    tracker_.release();
  }
}

void Tracking::rectifyTracker(const cv::Mat& mat, const cv::Rect2d& rect)
{
  if (!tracker_.empty())
  {
    tracker_.release();
  }
  tracker_ = cv::Tracker::create("MIL");
  tracker_->init(mat, rect);
  rect_ = rect;
}

bool Tracking::updateTracker(const cv::Mat& mat)
{
  bool ret = tracker_->update(mat, rect_);
  ageing_++;
  return ret;
}

cv::Rect2d Tracking::getRect()
{
  return rect_;
}

std::string Tracking::getObjName()
{
  return obj_name_;
}

int32_t Tracking::getTrackingId()
{
  return tracking_id_;
}

bool Tracking::isActive()
{
  return ageing_ < kAgeingThreshold;
}

bool Tracking::isDetected()
{
  return detected_;
}

void Tracking::clearDetected()
{
  detected_ = false;
}

void Tracking::setDetected()
{
  ageing_ = 0;
  detected_ = true;
}

}  // namespace tracker
}  // namespace object_analytics_nodelet
