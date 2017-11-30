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

#include <ros/console.h>

#include "object_analytics_nodelet/model/object_utils.h"

namespace object_analytics_nodelet
{
namespace model
{
void ObjectUtils::fill2DObjects(const ObjectsInBoxes::ConstPtr& objects_in_boxes2d, Object2DVector& objects2d)
{
  for (auto item : objects_in_boxes2d->objects_vector)
  {
    Object2D object2d(item);
    objects2d.push_back(object2d);
  }
}

void ObjectUtils::fill3DObjects(const ObjectsInBoxes3D::ConstPtr& objects_in_boxes3d, Object3DVector& objects3d)
{
  for (auto item : objects_in_boxes3d->objects_in_boxes)
  {
    Object3D object3d(item);
    objects3d.push_back(object3d);
  }
}

void ObjectUtils::findMaxIntersectionRelationships(const Object2DVector& objects2d, Object3DVector& objects3d,
                                                   RelationVector& relations)
{
  for (auto obj2d : objects2d)
  {
    Object3DVector::iterator max_it = objects3d.begin();
    double max = 0;

    auto obj2d_roi = obj2d.getRoi();
    const cv::Rect2d rect2d(obj2d_roi.x_offset, obj2d_roi.y_offset, obj2d_roi.width, obj2d_roi.height);
    for (Object3DVector::iterator it = max_it; it != objects3d.end(); ++it)
    {
      auto obj3d_roi = it->getRoi();
      const cv::Rect2d rect3d(obj3d_roi.x_offset, obj3d_roi.y_offset, obj3d_roi.width, obj3d_roi.height);
      auto area = getMatch(rect2d, rect3d);

      if (area < max)
      {
        continue;
      }

      max = area;
      max_it = it;
    }

    if (max <= 0)
    {
      ROS_DEBUG_STREAM("Cannot find correlated 3D object for " << obj2d);
      continue;
    }

    relations.push_back(Relation(obj2d, *max_it));
    objects3d.erase(max_it);
  }
}

void ObjectUtils::getMinMaxPointsInX(const PointCloudT::ConstPtr& point_cloud, PointT& x_min, PointT& x_max)
{
  auto cmp_x = [](PointT const& l, PointT const& r) { return l.x < r.x; };
  auto minmax_x = std::minmax_element(point_cloud->begin(), point_cloud->end(), cmp_x);
  x_min = *(minmax_x.first);
  x_max = *(minmax_x.second);
}

void ObjectUtils::getMinMaxPointsInY(const PointCloudT::ConstPtr& point_cloud, PointT& y_min, PointT& y_max)
{
  auto cmp_y = [](PointT const& l, PointT const& r) { return l.y < r.y; };
  auto minmax_y = std::minmax_element(point_cloud->begin(), point_cloud->end(), cmp_y);
  y_min = *(minmax_y.first);
  y_max = *(minmax_y.second);
}

void ObjectUtils::getMinMaxPointsInZ(const PointCloudT::ConstPtr& point_cloud, PointT& z_min, PointT& z_max)
{
  auto cmp_z = [](PointT const& l, PointT const& r) { return l.z < r.z; };
  auto minmax_z = std::minmax_element(point_cloud->begin(), point_cloud->end(), cmp_z);
  z_min = *(minmax_z.first);
  z_max = *(minmax_z.second);
}

double ObjectUtils::getMatch(const cv::Rect2d& r1, const cv::Rect2d& r2)
{
  cv::Rect2i ir1(r1), ir2(r2);
  /* calculate center of rectangle #1*/
  cv::Point2i c1(ir1.x + (ir1.width >> 1), ir1.y + (ir1.height >> 1));
  /* calculate center of rectangle #2*/
  cv::Point2i c2(ir2.x + (ir2.width >> 1), ir2.y + (ir2.height >> 1));

  double a1 = ir1.area(), a2 = ir2.area(), a0 = (ir1 & ir2).area();
  /* calculate the overlap rate*/
  double overlap = a0 / (a1 + a2 - a0);
  /* calculate the deviation between centers #1 and #2*/
  double deviate = sqrt(powf((c1.x - c2.x), 2) + powf((c1.y - c2.y), 2));

  /* calculate the match rate. The more overlap, the more matching. Contrary, the more deviation, the less matching*/
  return overlap * 100 / deviate;
}
}  // namespace model
}  // namespace object_analytics_nodelet
