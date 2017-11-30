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

#include <algorithm>
#include <vector>

#include <ros/console.h>

#include "object_analytics_nodelet/model/object3d.h"
#include "object_analytics_nodelet/model/object_utils.h"
#include "object_analytics_nodelet/model/projector.h"

namespace object_analytics_nodelet
{
namespace model
{
Object3D::Object3D(const PointCloudT::ConstPtr& cloud, const std::vector<int>& indices,
                   const std::shared_ptr<Projector>& projector)
{
  PointCloudT::Ptr points(new PointCloudT(*cloud, indices));

  PointT x_min_point, x_max_point;
  ObjectUtils::getMinMaxPointsInX(points, x_min_point, x_max_point);
  min_.x = x_min_point.x;
  max_.x = x_max_point.x;

  PointT y_min_point, y_max_point;
  ObjectUtils::getMinMaxPointsInY(points, y_min_point, y_max_point);
  min_.y = y_min_point.y;
  max_.y = y_max_point.y;

  PointT z_min_point, z_max_point;
  ObjectUtils::getMinMaxPointsInZ(points, z_min_point, z_max_point);
  min_.z = z_min_point.z;
  max_.z = z_max_point.z;

  auto pointT2point32 = [](PointT point) -> geometry_msgs::Point32 {  // NOLINT
    geometry_msgs::Point32 point32;
    point32.x = point.x;
    point32.y = point.y;
    point32.z = point.z;
    return point32;
  };
  int useless, x_start, y_start, x_end, y_end;
  projector->project3dToPixel(pointT2point32(x_min_point), x_start, useless);
  projector->project3dToPixel(pointT2point32(y_min_point), useless, y_start);
  projector->project3dToPixel(pointT2point32(x_max_point), x_end, useless);
  projector->project3dToPixel(pointT2point32(y_max_point), useless, y_end);
  if (x_start < 0)
  {
    roi_.x_offset = 0; /**< Due to measurement error and downsampling, set small minus value to zero */
  }
  else
  {
    roi_.x_offset = x_start;
  }

  if (y_start < 0)
  {
    roi_.y_offset = 0; /**< Due to measurement error and downsampling, set small minus value to zero */
  }
  else
  {
    roi_.y_offset = y_start;
  }
  if (x_end < x_start)
  {
    roi_.width = 0; /**< Due to measurement error and downsampling, set small minus value to zero */
  }
  else
  {
    roi_.width = x_end - x_start;
  }

  if (y_end < y_start)
  {
    roi_.height = 0; /**< Due to measurement error and downsampling, set small minus value to zero */
  }
  else
  {
    roi_.height = y_end - y_start;
  }
}

Object3D::Object3D(const object_analytics_msgs::ObjectInBox3D& object3d)
  : roi_(object3d.roi), min_(object3d.min), max_(object3d.max)
{
}

std::ostream& operator<<(std::ostream& os, const Object3D& obj)
{
  os << "Object3D[min=" << obj.min_.x << "," << obj.min_.y << "," << obj.min_.z;
  os << " max=" << obj.max_.x << "," << obj.max_.y << "," << obj.max_.z << ", roi=" << obj.roi_.x_offset << ","
     << obj.roi_.y_offset << "," << obj.roi_.width << "," << obj.roi_.height << "]";
  return os;
}
}  // namespace model
}  // namespace object_analytics_nodelet
