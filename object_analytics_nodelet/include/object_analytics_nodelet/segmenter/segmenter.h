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

#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "object_analytics_nodelet/model/object2d.h"
#include "object_analytics_nodelet/model/object3d.h"
#include "object_analytics_nodelet/segmenter/algorithm_provider.h"

namespace object_analytics_nodelet
{
namespace segmenter
{
using object_analytics_nodelet::model::PointT;
using object_analytics_nodelet::model::PointCloudT;

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
   * @param[in] nh        Ros node handle
   */
  explicit Segmenter(ros::NodeHandle& nh);

  /** Default desctructor */
  ~Segmenter() = default;

private:
  void cbSegment(const sensor_msgs::PointCloud2::ConstPtr&);
  void getPclPointCloud(const sensor_msgs::PointCloud2::ConstPtr&, PointCloudT&);
  void segment(const PointCloudT::ConstPtr&, Object3DVector&);
  void publishResult(const std_msgs::Header&, const Object3DVector&);

  ros::Subscriber sub_;
  ros::Publisher pub_;
  std::unique_ptr<object_analytics_nodelet::segmenter::AlgorithmProvider> provider_;
};
}  // namespace segmenter
}  // namespace object_analytics_nodelet
#endif  // OBJECT_ANALYTICS_NODELET_SEGMENTER_SEGMENTER_H
