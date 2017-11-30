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

#include "object_analytics_nodelet/const.h"
#include "object_analytics_nodelet/model/object_utils.h"
#include "object_analytics_nodelet/merger/merger.h"

using object_analytics_msgs::ObjectInBox3D;
using object_analytics_nodelet::model::ObjectUtils;

namespace object_analytics_nodelet
{
namespace merger
{
const int Merger::kMsgQueueSize = 100;

Merger::Merger(ros::NodeHandle& nh)
{
  pub_result_ = nh.advertise<ObjectsInBoxes3D>(Const::kTopicLocalization, 10);
  sub_2d_ = std::unique_ptr<Subscriber2D>(new Subscriber2D(nh, Const::kTopicDetection, kMsgQueueSize));
  sub_3d_ = std::unique_ptr<Subscriber3D>(new Subscriber3D(nh, Const::kTopicSegmentation, kMsgQueueSize));
  sub_sync_ = std::unique_ptr<ApproximateSynchronizer2D3D>(
      new ApproximateSynchronizer2D3D(ApproximatePolicy(kMsgQueueSize), *sub_2d_, *sub_3d_));
  sub_sync_->registerCallback(boost::bind(&Merger::cbMerge, this, _1, _2));
}

void Merger::cbMerge(const ObjectsInBoxes::ConstPtr& objects_in_boxes2d,
                     const ObjectsInBoxes3D::ConstPtr& objects_in_boxes3d)
{
  Object2DVector objects2d;
  ObjectUtils::fill2DObjects(objects_in_boxes2d, objects2d);
  Object3DVector objects3d;
  ObjectUtils::fill3DObjects(objects_in_boxes3d, objects3d);
  RelationVector relations;
  ObjectUtils::findMaxIntersectionRelationships(objects2d, objects3d, relations);
  publishResult(relations, objects_in_boxes2d->header);  // NOTE: MUST use detection timstamp
}

void Merger::publishResult(const RelationVector& relations, const std_msgs::Header& header)
{
  boost::shared_ptr<ObjectsInBoxes3D> msgs = boost::make_shared<ObjectsInBoxes3D>();
  msgs->header = header;

  for (auto item : relations)
  {
    ObjectInBox3D obj3d;
    obj3d.roi = item.first.getRoi();
    obj3d.min = item.second.getMin();
    obj3d.max = item.second.getMax();
    msgs->objects_in_boxes.push_back(obj3d);
  }

  pub_result_.publish(msgs);
}
}  // namespace merger
}  // namespace object_analytics_nodelet
