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

#ifndef OBJECT_ANALYTICS_NODELET_TRACKER_TRACKING_NODELET_H
#define OBJECT_ANALYTICS_NODELET_TRACKER_TRACKING_NODELET_H

#include <vector>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <object_msgs/ObjectsInBoxes.h>
#include <object_analytics_msgs/TrackedObjects.h>
#include "object_analytics_nodelet/tracker/tracking_manager.h"
#include "object_analytics_nodelet/tracker/tracking.h"

namespace object_analytics_nodelet
{
namespace tracker
{
/** @class TrackingNodelet
 * ROS Nodelet of multiple trackings, each against one object detected across camera frames.
 *
 * - Subscribe topics
 *   - /object_analytics/rgb. This class listen to "sensor_msgs::Image" published by an RGBD camera, see @ref rgb_cb().
 *   - /object_analytics/detection. This class also listen to "object_msgs::ObjectsInBoxes" published by object
 * detection nodelet,
 * see @ref obj_cb().
 * - Publish topic
 *   - /object_analytics/tracking. This class publish "object_analytics_msgs::TrackedObjects", see @ref
 * tracking_publish().
 *
 * The tracking workflow is initiated by a detection frame. Roi of each detected object will be used to initialize a
 * tracker for that object. Then the tracker will be kept updated with each successive frames arrived, we calling them
 * tracking frames, till next detection frame arrives. So the rhythm of the workflow is to repeat this sequence:
 *
 * [detection, tracking, tracking, tracking, ..., tracking]
 *
 * It pending on the frequency of detection, several tracking frames are processed in between. The faster the tracker
 * algorithm or the less objects to track, the more tracking frames being processed before the next detection frame
 * arrives.
 *
 * TrackingNodelet simply buffers the RGB image in @ref rgb_cb(), and waiting for the arrival of the detection frame.
 * Then in @ref obj_cb(), TrackingNodelet noted down this object frame and last object frame. The "last" object frame is
 * used to initiate trackings, then TrackingNodelet will process all successive frames before "this" object frame.
 *
 * TrackingNodelet has a @ref TrackingManager to process tracking updates from both detection frames and tracking
 * frames.
 */
class TrackingNodelet : public nodelet::Nodelet
{
public:
  /**
   * @brief Init this TrackingNodelet.
   */
  void onInit();

  /**
   * @brief Default destructor.
   */
  ~TrackingNodelet();

private:
  /**
   * @brief Callback from the object detection.
   *
   * @param[in] objs List of objects detected in a detection frame.
   */
  void obj_cb(const object_msgs::ObjectsInBoxesConstPtr& objs);

  /**
   * @brief Callback from the rgb image.
   *
   * @param[in] img Image frame captured by camera.
   */
  void rgb_cb(const sensor_msgs::ImageConstPtr& img);

  /**
   * @brief Publish tracked objects.
   *
   * @param[in] header Message header of the tracked objects.
   */
  void tracking_publish(const std_msgs::Header& header);

  ros::Publisher pub_tracking_;                             /**< Tracking publisher.*/
  ros::Subscriber sub_rgb_;                                 /**< Rgb image subscriber.*/
  ros::Subscriber sub_obj_;                                 /**< Object detection subscriber.*/
  std::shared_ptr<TrackingManager> tm_;                     /**< TrackingManager*/
  std::vector<sensor_msgs::ImageConstPtr> rgbs_;            /**< Rgb image buffer.*/
  object_msgs::ObjectsInBoxesConstPtr last_obj_, this_obj_; /**< Last detection frame, and this detection frame.*/
  ros::Time last_detection_, this_detection_;               /**< Timestamp of last and this detection frame.*/
};
}  // namespace tracker
}  // namespace object_analytics_nodelet
#endif  // OBJECT_ANALYTICS_NODELET_TRACKER_TRACKING_NODELET_H
