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

#ifndef OBJECT_ANALYTICS_NODELET_SPLITTER_SPLITTER_H
#define OBJECT_ANALYTICS_NODELET_SPLITTER_SPLITTER_H

#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>

#include "object_analytics_nodelet/model/projector.h"

namespace object_analytics_nodelet
{
namespace splitter
{
/** @class Splitter
 * @brief Implementaion of splitter logic.
 *
 * Subscrib PointCloud2 type topic which contains both 3d point cloud and rgb image, separate image and 3d point cloud
 * and re-publish.
 */
class Splitter
{
public:
  /*
   * Constructor
   *
   * @param[in] nh  Ros node handle
   */
  explicit Splitter(ros::NodeHandle& nh);

  /** Default destructor */
  ~Splitter() = default;

private:
  /**
   * @brief Callback for point cloud topic.
   *
   * Point cloud which must also contain rgb image information will be splitted into two types messages - rgb image and
   * publish for 2d detection use, point cloud same as the input and publish for 3d segmentaion.
   *
   * @param[in] points Point cloud with rgb image from rgb-d sensor
   */
  void cbSplit(const sensor_msgs::PointCloud2::ConstPtr& points);

  /**
   * Callback for frequency control topic on segmentation
   *
   * @param[in] header Topic content
   */
  void cbSegmentationFlag(const std_msgs::Header::ConstPtr& header);

  /**
   * Callback for frequency control topic on tracking
   *
   * @param[in] header Topic content
   */
  void cbTrackingFlag(const std_msgs::Header::ConstPtr& header);

  ros::NodeHandle nh_;
  ros::Publisher pub_2d_;
  ros::Publisher pub_3d_;
  ros::Subscriber sub_pc2_;
  std::shared_ptr<object_analytics_nodelet::model::Projector> projector_;
};
}  // namespace splitter
}  // namespace object_analytics_nodelet
#endif  // OBJECT_ANALYTICS_NODELET_SPLITTER_SPLITTER_H
