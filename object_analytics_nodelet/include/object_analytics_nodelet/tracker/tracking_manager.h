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

#ifndef OBJECT_ANALYTICS_NODELET_TRACKER_TRACKING_MANAGER_H
#define OBJECT_ANALYTICS_NODELET_TRACKER_TRACKING_MANAGER_H

#include <string>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <object_msgs/ObjectsInBoxes.h>
#include <object_analytics_msgs/TrackedObjects.h>
#include "object_analytics_nodelet/tracker/tracking.h"

namespace object_analytics_nodelet
{
namespace tracker
{
/** @class TrackingManager
 * Manage multiple trackings, each against one object detected across camera frames.
 *
 * TrackingManager owns a list of trackings. A tracking is added to the list when it is initially detected, see
 * @ref detect(). Then in the successive frames, every tracking is updated independently, see @ref track().
 *
 * For an object detected, TrackingManager search the existing list if it is tracked already. A detected object
 * has its own object name (e.g. person, dog, etc.) and a roi (region of interest, also known as bounding box). The
 * roi's matching level is measured by its overlapping rate and its centor deviation, see @ref
 * model::ObjectUtils::getMatch(). A @ref kMatchThreshold is used as the minimum matching level of roi. By matching
 * the object name and the roi, see @ref getTracking(), TrackingManager find an existing tracking for this object. Or
 * a new tracking should be added, see @ref addTracking().
 *
 * TrackingManager maintains a @ref tracking_cnt. When adding a new tracking, the value of tracking_cnt will be
 * assigned to that tracking, as a unique ID across frames. Then the tracking_cnt automatically increased by one.
 *
 * TrackingManager also maintains a @ref kProbabilityThreshold, only when detected with a confidence level not less
 * than this threshold will the object be added to the tracking list. This is necessary to mask any unexpected or
 * unstable detection results.
 *
 * For a tracked object, TrackingManager keep updating tracking roi in the successive tracking frames, even this object
 * might not be detected in those frames. The tracking is considered "active", till its @ref aging_ reaches a threshold.
 * TrackingManager maintains a @kAgeingThreshold specify the maximum age of an active tracking. Trackings below this
 * age are actively updated when tracking frame arrives. Trackings above this age are consider inactive, and to be
 * removed from the list.

 * Paralleling computation is enabled in TrackingManager, supported by openmp from compilers. @ref kNumOfThread
 * specifies the number of threads used for paralleling computation. Usually this should be less than the maximum
 * number of threads supported by the platform.
 */
class TrackingManager
{
public:
  /**
   * @brief Constructor, a TrackingManager shall be created for one stream.
   *
   * @param[in] pnh Private node handler
   */
  explicit TrackingManager(ros::NodeHandle &pnh);

  /**
   * @brief Default destructor.
   */
  ~TrackingManager();

  /**
   * @brief Manage trackings when objects detected from a new frame.
   *
   * For each object detected, TrackingManager will search the list if this object has been tracked already.
   * This is done by @ref getTracking(). If tracking does not exist for this object, a new tracking will be added.
   *
   * Only when detected with a confidence level not less than @ref kProbabilityThreshold will the object be marked
   * as "Detected" in this function, see @ref Tracking::setDetected().
   *
   * For all "Detected" objects, their trackers will be rectified with the detection rois, see @ref
   * Tracking::rectifyTracker().
   *
   * Finally, inactive trackings shall be removed from the list, see @ref cleanTrackings().
   *
   * @param[in] mat A new frame.
   * @param[in] objs Objects detected from this frame.
   */
  void detect(const cv::Mat& mat, const object_msgs::ObjectsInBoxesConstPtr& objs);

  /**
   * @brief Manage trackings when a new frame arrives.
   *
   * When a new frame arrives, for all existing trackings, TrackingManager will update their trackers, each calculating
   * a new roi.
   *
   * @param[in] mat A new frame.
   */
  void track(const cv::Mat& mat);

  /**
   * @brief Get Tracked objects list.
   *
   * Only objects marked as "Detected" in the latest @ref detect() shall be returned.
   *
   * @param[out] objs List of tracked objects.
   * @return Count of tracked objects.
   */
  int32_t getTrackedObjs(const object_analytics_msgs::TrackedObjectsPtr& objs);

private:
  static const int32_t kAgeingThreshold;    /**< The default aging threshold*/
  static const float kMatchThreshold;       /**< The minimum matching level of roi*/
  static const int32_t kNumOfThread;        /**< Number of threads used for paralleling computation*/
  static const float kProbabilityThreshold; /**< The default probability threshold*/
  static const int32_t kSamplerInputRadius; /**< Input radius parameter for tracker's sampling (from OpenCV)*/
  static int32_t tracking_cnt;              /**< Count of trackings, as a unique ID of a same object across frames*/
  int32_t aging_th_;                        /**< The effective aging threshold*/
  float probability_th_;                    /**< The effective probability threshold*/
  std::vector<std::shared_ptr<Tracking>> trackings_; /**< List of trackings, each for one detected object*/

  /**
   * @brief Add a new tracking to the list.
   *
   * The current value of @ref tracking_cnt will be assigned to this new tracking, which denotes a unique ID
   * of the same object across frames. Then the tracking_cnt automatically increases by one.
   *
   * @param[in] obj_name Name of the object (e.g. people, dog, etc.).
   * @param[in] rect Roi of the tracked object.
   * @return Pointer to the tracking added.
   */
  std::shared_ptr<Tracking> addTracking(const std::string& obj_name, const cv::Rect2d& rect);

  /**
   * @brief Clean up inactive tracking in the list. @ref kAgeingThreshold specify the maximum age of an active tracking.
   * Trackings below this age are actively updated when tracking frame arrives. Trackings above this age are consider
   * inactive, and to be removed from the list.
   */
  void cleanTrackings();

  /**
   * @brief Get the most matching tracking from the list.
   *
   * For a detected object, this function returns the most matching tracking, with the same object name, and the most
   * matching roi measured by @ref model::ObjectUtils::getMatch(), not less than @ref kMatchThreshold.
   *
   * @param[in] obj_name Name of the object (e.g. people, dog, etc.).
   * @param[in] roi Bounding box of the object.
   * @return Pointer to the tracking matched. An empty pointer if none tracking matched.
   */
  std::shared_ptr<Tracking> getTracking(const std::string& obj_name, const cv::Rect2d& roi);

  /**
   * @brief Validate the ROI against the size of an image array.
   *
   * In Debug build, ROS_ASSERT failure will be raised in case unexpected ROI detected.
   * In Release build, false shall be returned in case unexpected ROI detected.
   *
   * @param[in] mat Input image arrary
   * @param[in] droi ROI
   * @return true if ROI valid, otherwise false.
   */
  bool validateROI(const cv::Mat& mat, const sensor_msgs::RegionOfInterest& droi);
};
}  // namespace tracker
}  // namespace object_analytics_nodelet
#endif  // OBJECT_ANALYTICS_NODELET_TRACKER_TRACKING_MANAGER_H
