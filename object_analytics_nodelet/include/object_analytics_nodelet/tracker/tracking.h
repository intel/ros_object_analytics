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

#ifndef OBJECT_ANALYTICS_NODELET_TRACKER_TRACKING_H
#define OBJECT_ANALYTICS_NODELET_TRACKER_TRACKING_H

#include <string>
#include <opencv2/tracking.hpp>

namespace object_analytics_nodelet
{
namespace tracker
{
/** @class Tracking
 * One single tracking against an object detected across camera frames.
 *
 * A tracking has a few external attributes
 * - tracking id, a unique ID denoting a same object arcoss frames.
 * - object name, name of the tracked object.
 * - rect, roi of the tracked object.
 *
 * Also a tracking has some internal attributs
 * - ageing, indicating how old since last detection frame. Reset to zero when object detected in the frame, and
 * increase upon every tracking frame arrives. @ref kAgeingThreshold specify the maximum age of an active tracking.
 * - detected flag, will be set when a detection frame arrives. True if the tracked object is detected in the frame.
 *
 * When a tracking is created, it is assigned a tracking ID, and associated with the name and roi of the detected
 * object. When a detection frame arrives, a tracking shall rectify its tracker, see @ref rectifyTracker, with the
 * detection roi. While when a tracking frame arrives, a tracking shall update its tracker.
 */
class Tracking
{
public:
  /**
   * @brief Constructor of Tracking.
   *
   * @param[in] tracking_id ID of this tracking.
   * @param[in] name Name of the tracked object.
   * @param[in] rect Roi of the tracked object.
   */
  explicit Tracking(int32_t tracking_id, const std::string& name, const cv::Rect2d& rect);

  /**
   * @brief Default destructor.
   */
  ~Tracking();

  /**
   * @brief Rectify tracker with the roi of the detected object.
   *
   * @param[in] mat The detection frame.
   * @param[in] rect Roi of the detected object.
   */
  void rectifyTracker(const cv::Mat& mat, const cv::Rect2d& rect);

  /**
   * @brief Update tracker with the tracking frame.
   *
   * @param[in] mat The tracking frame.
   * @return true if tracker was updated successfully, otherwise false.
   */
  bool updateTracker(const cv::Mat& mat);

  /**
   * @brief Get the roi of tracked object.
   *
   * @return Roi of the tracked object.
   */
  cv::Rect2d getRect();

  /**
   * @brief Get the name of the tracked object.
   *
   * @return Name of the tracked object.
   */
  std::string getObjName();

  /**
   * @brief Get the tracking id.
   *
   * @return ID of the tracking.
   */
  int32_t getTrackingId();

  /**
   * @brief Get the aging.
   *
   * @return Aging of the tracking.
   */
  int32_t getAging();

  /**
   * @brief Get the detected status of a tracking.
   *
   * @return true if tracking is detected, otherwise false.
   */
  bool isDetected();

  /**
   * @brief Set the detected status of a tracking. Ageing is set to zero also.
   */
  void setDetected();

  /**
   * @brief Clear the detected status of a tracking.
   */
  void clearDetected();

private:
  cv::Ptr<cv::Tracker> tracker_;         /**< Tracker associated to this tracking.*/
  cv::Rect2d rect_;                      /**< Roi of the tracked object.*/
  std::string obj_name_;                 /**< Name of the tracked object.*/
  int32_t tracking_id_;                  /**< ID of this tracking.*/
  int32_t ageing_;                       /**< Age of this tracking.*/
  bool detected_;                        /**< Detected status of this tracking.*/
};
}  // namespace tracker
}  // namespace object_analytics_nodelet
#endif  // OBJECT_ANALYTICS_NODELET_TRACKER_TRACKING_H
