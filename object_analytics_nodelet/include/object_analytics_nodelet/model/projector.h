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

#ifndef OBJECT_ANALYTICS_NODELET_MODEL_PROJECTOR_H
#define OBJECT_ANALYTICS_NODELET_MODEL_PROJECTOR_H

#include <geometry_msgs/Point32.h>

namespace object_analytics_nodelet
{
namespace model
{
/** @class Projector
 * @brief Wrapper of object_analytics_msgs::ObjectInBox3D.
 *
 * There are two scenarios of using this class. One is in SegmenterNodelet, this class is used to hold the result of 3d
 * segmentation and calculate minimum and maximum point in 3d space. Another one is in MergerNodelet, it's build based
 * on segmentation result.
 */
class Projector
{
public:
  /** Default destrcutor */
  virtual ~Projector() = default;

  /**
   * Interface to project object in 3d space into pixel space.
   *
   * @param[in]   point   3d point to be projected on pixel space
   * @param[out]  x       X in pixel
   * @param[out]  y       Y in pixel
   */
  virtual void project3dToPixel(const geometry_msgs::Point32& point, int& x, int& y) = 0;
};
}  // namespace model
}  // namespace object_analytics_nodelet
#endif  // OBJECT_ANALYTICS_NODELET_MODEL_PROJECTOR_H
