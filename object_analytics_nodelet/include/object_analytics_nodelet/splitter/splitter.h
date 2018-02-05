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

#define PCL_NO_PRECOMPILE
#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

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
  /** Default constructor */
  Splitter() = default;

  /** Default destructor */
  ~Splitter() = default;

  /**
   * @brief Split PointCloud2 w/ RGB into Image.
   *
   * param[in]      points  Pointer to PointCloud2 w/ RGB
   * param[in,out]  image   Pointer to Image
   */
  static void split(const sensor_msgs::PointCloud2::ConstPtr& points, sensor_msgs::Image::Ptr& image);
};
}  // namespace splitter
}  // namespace object_analytics_nodelet
#endif  // OBJECT_ANALYTICS_NODELET_SPLITTER_SPLITTER_H
