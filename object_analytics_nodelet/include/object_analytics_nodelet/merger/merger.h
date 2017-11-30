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

#ifndef OBJECT_ANALYTICS_NODELET_MERGER_MERGER_H
#define OBJECT_ANALYTICS_NODELET_MERGER_MERGER_H

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <object_analytics_msgs/ObjectsInBoxes3D.h>
#include "object_analytics_nodelet/model/object_utils.h"

namespace object_analytics_nodelet
{
namespace merger
{
using object_analytics_msgs::ObjectsInBoxes3D;

/** @class Merger
 * @brief Implementaion of merge logic.
 *
 * Merge 3d segmentation and 2d detection results together and publish the 3d localization result.
 * First using message header's timestamp, synchronize 3d segmentation result which is subscribed on "segmentation"
 * topic and 2d detection result which is subscribed on "detection" topic. Then find corresponding 3d segmentaion who
 * has the maximum overlap area in 2d for each 2d detection result. Finally publish the merged localization result.
 */
class Merger
{
public:
  /**
   * Constructor
   *
   * @param[in] nh    Ros node handle.
   */
  explicit Merger(ros::NodeHandle& nh);

  /** Default destructor */
  ~Merger() = default;

private:
  /**
   * @brief TimeSynchronizer callback method.
   *
   * This method should be registered as callback function of TimeSynchronizer for detection topic and segmentation
   * topic.
   *
   * @param[in] objects_in_boxes2d    Pointer to 2d detection result
   * @param[in] objects_in_boxes3d    Pointer to 3d segmentation result
   */
  void cbMerge(const ObjectsInBoxes::ConstPtr& objects_in_boxes2d,
               const ObjectsInBoxes3D::ConstPtr& objects_in_boxes3d);

  /**
   * Publish merged result on localization topic.
   *
   * @param[in] relations   Pair list of 2d object and corresponding 3d object
   * @param[in] header      The header of message
   */
  void publishResult(const RelationVector& relations, const std_msgs::Header& header);

  using Subscriber2D = message_filters::Subscriber<ObjectsInBoxes>;
  using Subscriber3D = message_filters::Subscriber<ObjectsInBoxes3D>;
  using ApproximatePolicy = message_filters::sync_policies::ApproximateTime<ObjectsInBoxes, ObjectsInBoxes3D>;
  using ApproximateSynchronizer2D3D = message_filters::Synchronizer<ApproximatePolicy>;

  ros::Publisher pub_result_;
  std::unique_ptr<Subscriber2D> sub_2d_;
  std::unique_ptr<Subscriber3D> sub_3d_;
  std::unique_ptr<ApproximateSynchronizer2D3D> sub_sync_;

  static const int kMsgQueueSize;
};
}  // namespace merger
}  // namespace object_analytics_nodelet
#endif  // OBJECT_ANALYTICS_NODELET_MERGER_MERGER_H
