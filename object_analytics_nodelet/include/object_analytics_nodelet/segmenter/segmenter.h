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

#ifndef OBJECT_ANALYTICS_NODELET_SEGMENTER_SEGMENTER_H
#define OBJECT_ANALYTICS_NODELET_SEGMENTER_SEGMENTER_H

#define PCL_NO_PRECOMPILE
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "object_analytics_msgs/ObjectsInBoxes3D.h"
#include "object_analytics_nodelet/model/object2d.h"
#include "object_analytics_nodelet/model/object3d.h"
#include "object_analytics_nodelet/segmenter/algorithm_provider.h"

namespace object_analytics_nodelet
{
namespace segmenter
{
using object_analytics_msgs::ObjectsInBoxes3D;
using object_analytics_nodelet::model::PointT;
using object_analytics_nodelet::model::PointCloudT;
using object_analytics_nodelet::segmenter::AlgorithmProvider;

/** @class Segmenter
 * Segmenter implmentation. Segment the coming point cloud into individual objects and publish on segmentation topic.
 */
class Segmenter
{
public:
  using Object3DVector = std::vector<object_analytics_nodelet::model::Object3D>;

  /**
   * Constructor
   *
   * @param[in] provider Pointer to AlgorithmProvider instance
   */
  explicit Segmenter(std::unique_ptr<AlgorithmProvider> provider);

  /** Default desctructor */
  ~Segmenter() = default;

  /**
   * @brief Do segmentation on given PointCloud2 and return ObjectsInBoxes3D message back.
   *
   * @param[in]     points  Pointer point to PointCloud2 message from sensor.
   * @param[in,out] msg     Pointer pint to ObjectsInBoxes3D message to take back.
   */
  void segment(const sensor_msgs::PointCloud2::ConstPtr& points, boost::shared_ptr<ObjectsInBoxes3D>& msg);

private:
  void getPclPointCloud(const sensor_msgs::PointCloud2::ConstPtr&, PointCloudT&);
  void doSegment(const PointCloudT::ConstPtr&, Object3DVector&);
  void composeResult(const Object3DVector&, boost::shared_ptr<ObjectsInBoxes3D>&);

  std::unique_ptr<AlgorithmProvider> provider_;
};
}  // namespace segmenter
}  // namespace object_analytics_nodelet
#endif  // OBJECT_ANALYTICS_NODELET_SEGMENTER_SEGMENTER_H
