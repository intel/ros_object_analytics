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

namespace object_analytics_nodelet
{
namespace model
{
Object3D::Object3D(const PointCloudT::ConstPtr& cloud, const std::vector<int>& indices)
{
  pcl::PointCloud<PointXYZPixel>::Ptr seg(new pcl::PointCloud<PointXYZPixel>);
  ObjectUtils::copyPointCloud(cloud, indices, seg);

  PointXYZPixel x_min_point, x_max_point;
  ObjectUtils::getMinMaxPointsInX(seg, x_min_point, x_max_point);
  min_.x = x_min_point.x;
  max_.x = x_max_point.x;

  PointXYZPixel y_min_point, y_max_point;
  ObjectUtils::getMinMaxPointsInY(seg, y_min_point, y_max_point);
  min_.y = y_min_point.y;
  max_.y = y_max_point.y;

  PointXYZPixel z_min_point, z_max_point;
  ObjectUtils::getMinMaxPointsInZ(seg, z_min_point, z_max_point);
  min_.z = z_min_point.z;
  max_.z = z_max_point.z;

  ObjectUtils::getProjectedROI(seg, this->roi_);
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
