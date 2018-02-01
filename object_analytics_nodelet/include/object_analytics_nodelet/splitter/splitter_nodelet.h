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

#ifndef OBJECT_ANALYTICS_NODELET_SPLITTER_SPLITTER_NODELET_H
#define OBJECT_ANALYTICS_NODELET_SPLITTER_SPLITTER_NODELET_H

#include <nodelet/nodelet.h>

#include "object_analytics_nodelet/splitter/splitter.h"

namespace object_analytics_nodelet
{
namespace splitter
{
/** @class SplitterNodelet
 * Splitter nodelet, splitter implementation holder.
 */
class SplitterNodelet : public nodelet::Nodelet
{
public:
  /** Default destrucgtor */
  ~SplitterNodelet() = default;

private:
  /** Inherit from Nodelet class. Initialize Splitter instance. */
  virtual void onInit();

  /**
   * @brief Callback for point cloud topic.
   *
   * Point cloud which must also contain rgb image information will be splitted into two types messages - rgb image and
   * publish for 2d detection use, point cloud same as the input and publish for 3d segmentaion.
   *
   * @param[in] points Point cloud with rgb image from rgb-d sensor
   */
  void cbSplit(const sensor_msgs::PointCloud2::ConstPtr& points);

  ros::Publisher pub_2d_;
  ros::Publisher pub_3d_;
  ros::Subscriber sub_pc2_;
};
}  // namespace splitter
}  // namespace object_analytics_nodelet
#endif  // OBJECT_ANALYTICS_NODELET_SPLITTER_SPLITTER_NODELET_H
