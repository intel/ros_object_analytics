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
#define PCL_NO_PRECOMPILE
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>

#include "object_analytics_nodelet/model/object_utils.h"

#include "tests/unittest_util.h"

using geometry_msgs::Point32;
using sensor_msgs::RegionOfInterest;
using object_msgs::Object;
using object_msgs::ObjectInBox;
using object_msgs::ObjectsInBoxes;
using object_analytics_msgs::ObjectInBox3D;
using object_analytics_msgs::ObjectsInBoxes3D;
using object_analytics_nodelet::model::PointT;
using object_analytics_nodelet::model::PointCloudT;
using object_analytics_nodelet::model::Object2D;
using object_analytics_nodelet::model::Object3D;
using object_analytics_nodelet::model::ObjectUtils;

void readPointCloudFromPCD(const std::string& name, PointCloudT::Ptr& pointcloud)
{
  if (pcl::io::loadPCDFile<PointT>(name.c_str(), *pointcloud) == -1)
  {
    assert(false);
  }
}

Object getObject(const std::string name, const float probability)
{
  Object obj;
  obj.object_name = name;
  obj.probability = probability;
  return obj;
}

RegionOfInterest getRoi(int x_offset, int y_offset, int width, int height)
{
  RegionOfInterest roi;
  roi.x_offset = x_offset;
  roi.y_offset = y_offset;
  roi.width = width;
  roi.height = height;
  return roi;
}

ObjectInBox getObjectInBox(int x_offset, int y_offset, int width, int height, const std::string& name,
                           const float probability)
{
  ObjectInBox oib;
  oib.roi = getRoi(x_offset, y_offset, width, height);
  oib.object = getObject(name, probability);
  return oib;
}

Point32 getPoint32(float x, float y, float z)
{
  Point32 point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

ObjectInBox3D getObjectInBox3D(int x_offset, int y_offset, int width, int height, int min_x, int min_y, int min_z,
                               int max_x, int max_y, int max_z)
{
  ObjectInBox3D oib3d;
  oib3d.roi = getRoi(x_offset, y_offset, width, height);
  oib3d.max = getPoint32(max_x, max_y, max_z);
  oib3d.min = getPoint32(min_x, min_y, min_z);
  return oib3d;
}

bool operator==(const Object& left, const Object& right)
{
  if (left.object_name != right.object_name || left.probability != right.probability)
  {
    return false;
  }

  return true;
}

bool operator==(const RegionOfInterest& left, const RegionOfInterest& right)
{
  if (left.x_offset != right.x_offset || left.y_offset != right.y_offset || left.width != right.width ||
      left.height != right.height)
  {
    return false;
  }

  return true;
}

bool operator==(const ObjectInBox& left, const Object2D& right)
{
  if (left.object.object_name == right.getObject().object_name && left.roi == right.getRoi())
  {
    return true;
  }

  return false;
}

bool operator==(const Point32& left, const Point32& right)
{
  if (left.x != right.x || left.y != right.y || left.z != right.z)
  {
    return false;
  }

  return true;
}

bool operator==(const ObjectInBox3D& left, const Object3D& right)
{
  if (left.roi == right.getRoi() && left.max == right.getMax() && left.min == right.getMin())
  {
    return true;
  }

  return false;
}

bool operator==(const ObjectInBox3D& left, const ObjectInBox3D& right)
{
  if (left.roi == right.roi && left.max == right.max && left.min == right.min)
  {
    return true;
  }

  return false;
}

bool operator==(const Object2D& left, const Object2D& right)
{
  if (left.getRoi() == right.getRoi() && left.getObject().object_name == right.getObject().object_name)
  {
    return true;
  }

  return false;
}

bool operator==(const Object3D& left, const Object3D& right)
{
  if (left.getRoi() == right.getRoi() && left.getMax() == right.getMax() && left.getMin() == right.getMin())
  {
    return true;
  }

  return false;
}

PointT getPointT(float x, float y, float z)
{
  PointT p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

bool operator==(const PointT& left, const PointT& right)
{
  if (left.x != right.x || left.y != right.y || left.z != right.z)
  {
    return false;
  }
  return true;
}

bool operator==(const PointXYZPixel& left, const PointT& right)
{
  if (left.x != right.x || left.y != right.y || left.z != right.z)
  {
    return false;
  }
  return true;
}

Header createHeader(uint32_t seq, const ros::Time& stamp, const std::string& frame_id)
{
  Header header;
  header.seq = seq;
  header.stamp = stamp;
  header.frame_id = frame_id;
  return header;
}

bool operator==(const Header& left, const Header& right)
{
  if (left.seq != right.seq || left.stamp != right.stamp || left.frame_id != right.frame_id)
  {
    return false;
  }
  return true;
}
