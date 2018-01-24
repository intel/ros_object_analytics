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

#include "object_analytics_nodelet/const.h"
#include "object_analytics_nodelet/merger/merger_nodelet.h"

namespace object_analytics_nodelet
{
namespace merger
{
const int MergerNodelet::kMsgQueueSize = 100;

void MergerNodelet::onInit()
{
  ros::NodeHandle nh = getNodeHandle();
  pub_result_ = nh.advertise<ObjectsInBoxes3D>(Const::kTopicLocalization, 10);
  sub_2d_ = std::unique_ptr<Subscriber2D>(new Subscriber2D(nh, Const::kTopicDetection, kMsgQueueSize));
  sub_3d_ = std::unique_ptr<Subscriber3D>(new Subscriber3D(nh, Const::kTopicSegmentation, kMsgQueueSize));
  sub_sync_ = std::unique_ptr<ApproximateSynchronizer2D3D>(
      new ApproximateSynchronizer2D3D(ApproximatePolicy(kMsgQueueSize), *sub_2d_, *sub_3d_));
  sub_sync_->registerCallback(boost::bind(&MergerNodelet::cbMerge, this, _1, _2));
}

void MergerNodelet::cbMerge(const ObjectsInBoxes::ConstPtr& objects_in_boxes2d,
                            const ObjectsInBoxes3D::ConstPtr& objects_in_boxes3d)
{
  boost::shared_ptr<ObjectsInBoxes3D> msgs = Merger::merge(objects_in_boxes2d, objects_in_boxes3d);
  pub_result_.publish(msgs);
}
}  // namespace merger
}  // namespace object_analytics_nodelet
PLUGINLIB_EXPORT_CLASS(object_analytics_nodelet::merger::MergerNodelet, nodelet::Nodelet)
