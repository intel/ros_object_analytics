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

#ifndef OBJECT_ANALYTICS_NODELET_MERGER_MERGER_NODELET_H
#define OBJECT_ANALYTICS_NODELET_MERGER_MERGER_NODELET_H

#include <nodelet/nodelet.h>
#include <std_msgs/Header.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "object_analytics_nodelet/merger/merger.h"

namespace object_analytics_nodelet
{
namespace merger
{
/** @class MergerNodelet
 * Merger nodelet, merger implementation holder.
 */
class MergerNodelet : public nodelet::Nodelet
{
public:
  /** Default destructor */
  ~MergerNodelet() = default;

private:
  /** Inherit from Nodelet class. Initialize Merger instance. */
  virtual void onInit();

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

  static const int kMsgQueueSize;

  using Subscriber2D = message_filters::Subscriber<ObjectsInBoxes>;
  using Subscriber3D = message_filters::Subscriber<ObjectsInBoxes3D>;
  using ApproximatePolicy = message_filters::sync_policies::ApproximateTime<ObjectsInBoxes, ObjectsInBoxes3D>;
  using ApproximateSynchronizer2D3D = message_filters::Synchronizer<ApproximatePolicy>;

  ros::Publisher pub_result_;
  std::unique_ptr<Subscriber2D> sub_2d_;
  std::unique_ptr<Subscriber3D> sub_3d_;
  std::unique_ptr<ApproximateSynchronizer2D3D> sub_sync_;
};
}  // namespace merger
}  // namespace object_analytics_nodelet
#endif  // OBJECT_ANALYTICS_NODELET_MERGER_MERGER_NODELET_H
