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

#ifndef OBJECT_ANALYTICS_NODELET_MODEL_OBJECT2D_H
#define OBJECT_ANALYTICS_NODELET_MODEL_OBJECT2D_H

#include <object_msgs/ObjectInBox.h>

namespace object_analytics_nodelet
{
namespace model
{
/** @class Object2D
 * @brief Wrapper of object_msgs::ObjectInBox.
 *
 * Constructed from 2d detection result, represents one object_msgs::ObjectInBox instance.
 */
class Object2D
{
public:
  /**
   * Constructor
   *
   * @param[in] object_in_box   Object in box which comes from 2d detection result.
   */
  explicit Object2D(const object_msgs::ObjectInBox& object_in_box);

  /** Default destructor */
  ~Object2D() = default;

  /**
   * Get the region of interest in sensor_msgs::RegionOfInterest type of underlying object in image space.
   *
   * @return Underlying region of interest in image space
   */
  inline sensor_msgs::RegionOfInterest getRoi() const
  {
    return roi_;
  }

  /**
   * Get the underlying object_msgs::Object.
   *
   * @return The underlying object_msgs::Object
   */
  inline object_msgs::Object getObject() const
  {
    return object_;
  }

  /**
   * Overload operator << to dump information of underlying information.
   *
   * @param[in,out] os    Standard output stream
   * @param[in]     obj   Object to be dumped
   *
   * @return Standard output stream
   */
  friend std::ostream& operator<<(std::ostream& os, const Object2D& obj);

private:
  const sensor_msgs::RegionOfInterest roi_;
  const object_msgs::Object object_;
};

using Object2DPtr = std::shared_ptr<Object2D>;
using Object2DConstPtr = std::shared_ptr<Object2D const>;
}  // namespace model
}  // namespace object_analytics_nodelet
#endif  // OBJECT_ANALYTICS_NODELET_MODEL_OBJECT2D_H
