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

#include <opencv2/core/types.hpp>
#include <ros/console.h>

#include "object_analytics_nodelet/model/object2d.h"

namespace object_analytics_nodelet
{
namespace model
{
Object2D::Object2D(const object_msgs::ObjectInBox& oib) : roi_(oib.roi), object_(oib.object)
{
}

std::ostream& operator<<(std::ostream& os, const Object2D& obj)
{
  os << "Object2D[" << obj.object_.object_name;
  os << ", @(" << obj.roi_.x_offset << ", " << obj.roi_.y_offset << ")";
  os << ", width=" << obj.roi_.width << ", height=" << obj.roi_.height << "]";
  return os;
}
}  // namespace model
}  // namespace object_analytics_nodelet
