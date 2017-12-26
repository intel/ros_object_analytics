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
#include <vector>
#include <omp.h>
#include <cv_bridge/cv_bridge.h>
#include "object_analytics_nodelet/model/object_utils.h"
#include "object_analytics_nodelet/tracker/tracking_manager.h"

using object_analytics_nodelet::model::ObjectUtils;

namespace object_analytics_nodelet
{
namespace tracker
{
// TrackingManager class implementation

const float TrackingManager::kMatchThreshold = 0.2;
const float TrackingManager::kProbabilityThreshold = 0.5;
int32_t TrackingManager::tracking_cnt = 0;
const int32_t TrackingManager::kNumOfThread = 4;

TrackingManager::TrackingManager()
{
}

TrackingManager::~TrackingManager()
{
}

void TrackingManager::track(const cv::Mat& mat)
{
  uint32_t i;

  ROS_DEBUG("****tracked objects: %zu", trackings_.size());
/* get all tracking ROIs updated with new frame*/
#pragma omp parallel for num_threads(kNumOfThread)
  for (i = 0; i < trackings_.size(); i++)
  {
    if (!trackings_[i]->updateTracker(mat))
    {
      ROS_DEBUG("tracking[%d] false !!!", trackings_[i]->getTrackingId());
    }
  }
}

void TrackingManager::detect(const cv::Mat& mat, const object_msgs::ObjectsInBoxesConstPtr& objs)
{
  uint32_t i;

  for (auto t : trackings_)
  {
    t->clearDetected();
  }
  ROS_DEBUG("****detected objects: %zu", objs->objects_vector.size());
/* rectify tracking ROIs with detected ROIs*/
#pragma omp parallel for num_threads(kNumOfThread)
  for (i = 0; i < objs->objects_vector.size(); i++)
  {
    object_msgs::Object dobj = objs->objects_vector[i].object;
    if (dobj.probability < kProbabilityThreshold)
    {
      continue;
    }
    std::string n = dobj.object_name;
    sensor_msgs::RegionOfInterest droi = objs->objects_vector[i].roi;
    /* some trackers do not accept an ROI beyond the size of a Mat*/
    if (droi.x_offset + droi.width > static_cast<uint32_t>(mat.cols))
    {
      droi.width = mat.cols - droi.x_offset;
    }
    if (droi.y_offset + droi.height > static_cast<uint32_t>(mat.rows))
    {
      droi.height = mat.rows - droi.y_offset;
    }
    cv::Rect2d r = cv::Rect2d(droi.x_offset, droi.y_offset, droi.width, droi.height);
    ROS_DEBUG("detected %s [%d %d %d %d] %.0f%%", n.c_str(), droi.x_offset, droi.y_offset, droi.width, droi.height,
              dobj.probability * 100);
    std::shared_ptr<Tracking> t;
#pragma omp critical
    {
      /* get matched tracking with the detected object name (class) and its ROI*/
      t = getTracking(n, r);
      /* add tracking if new object detected*/
      if (!t)
      {
        t = addTracking(n, r);
      }
      t->setDetected();
    }

    /* rectify tracking ROI with detected ROI*/
    t->rectifyTracker(mat, r);
  }

  /* clean up inactive trackings*/
  cleanTrackings();
}

int32_t TrackingManager::getTrackedObjs(const object_analytics_msgs::TrackedObjectsPtr& objs)
{
  for (auto t : trackings_)
  {
    if (!t->isDetected())
    {
      continue;
    }
    object_analytics_msgs::TrackedObject tobj;
    cv::Rect2d r = t->getRect();
    tobj.id = t->getTrackingId();
    tobj.roi.x_offset = static_cast<int>(r.x);
    tobj.roi.y_offset = static_cast<int>(r.y);
    tobj.roi.width = static_cast<int>(r.width);
    tobj.roi.height = static_cast<int>(r.height);
    objs->tracked_objects.push_back(tobj);
  }
  ROS_DEBUG("****tracked objects: %zu published", objs->tracked_objects.size());

  return objs->tracked_objects.size();
}

std::shared_ptr<Tracking> TrackingManager::addTracking(const std::string& name, const cv::Rect2d& rect)
{
  std::shared_ptr<Tracking> t = std::make_shared<Tracking>(tracking_cnt++, name, rect);
  if (tracking_cnt == -1)
  {
    ROS_WARN("tracking count overflow");
  }
  ROS_DEBUG("addTracking[%d] +++", t->getTrackingId());
  trackings_.push_back(t);
  return t;
}

void TrackingManager::cleanTrackings()
{
  std::vector<std::shared_ptr<Tracking>>::iterator t = trackings_.begin();
  while (t != trackings_.end())
  {
    if (!(*t)->isActive())
    {
      ROS_DEBUG("removeTracking[%d] ---", (*t)->getTrackingId());
      t = trackings_.erase(t);
    }
    else
    {
      ++t;
    }
  }
}

/* get matched tracking for each detected object,
 * with the same object name,
 * and the most matching ROI
 */
std::shared_ptr<Tracking> TrackingManager::getTracking(const std::string& obj_name, const cv::Rect2d& drect)
{
  double match = 0;
  std::shared_ptr<Tracking> tracking = std::shared_ptr<Tracking>();

  /* searching over all trackings*/
  for (auto t : trackings_)
  {
    /* seek for the one with the same object name (class), and not yet rectified*/
    if (!t->isDetected() && 0 == obj_name.compare(t->getObjName()))
    {
      cv::Rect2d trect = t->getRect();
      double m = ObjectUtils::getMatch(trect, drect);
      ROS_DEBUG("tr[%d] %s [%d %d %d %d]%.2f", t->getTrackingId(), t->getObjName().c_str(), (int)trect.x, (int)trect.y,
                (int)trect.width, (int)trect.height, m);
      /* seek for the one with the most matching ROI*/
      if (m > match)
      {
        tracking = t;
        match = m;
      }
    }
  }
  /* if matching above the threshold, return the tracking*/
  if (match >= TrackingManager::kMatchThreshold)
  {
    return tracking;
  }
  else
  {
    return std::shared_ptr<Tracking>();
  }
}

}  // namespace tracker
}  // namespace object_analytics_nodelet
