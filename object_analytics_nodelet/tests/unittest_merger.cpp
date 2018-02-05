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
#include <cassert>
#include <gtest/gtest.h>
#include "object_analytics_nodelet/merger/merger.h"
#include "tests/unittest_util.h"

using object_analytics_nodelet::merger::Merger;

TEST(UnitTestMerger, merge_BothEmpty)
{
  ObjectsInBoxes::Ptr objects_in_boxes2d(new ObjectsInBoxes);
  std_msgs::Header header2D = createHeader(1, ros::Time(0, 1000), "camera_rgb_optical_frame");
  objects_in_boxes2d->header = header2D;

  ObjectsInBoxes3D::Ptr objects_in_boxes3d(new ObjectsInBoxes3D);
  std_msgs::Header header3D = createHeader(1, ros::Time(0, 1001), "camera_rgb_optical_frame");
  objects_in_boxes3d->header = header3D;

  ObjectsInBoxes3D::Ptr merged = Merger::merge(objects_in_boxes2d, objects_in_boxes3d);
  EXPECT_TRUE(merged->header == header2D);
  EXPECT_EQ(merged->objects_in_boxes.size(), 0);
}

TEST(UnitTestMerger, merge_Empty3D)
{
  ObjectsInBoxes::Ptr objects_in_boxes2d(new ObjectsInBoxes);
  std_msgs::Header header2D = createHeader(1, ros::Time(0, 1000), "camera_rgb_optical_frame");
  objects_in_boxes2d->header = header2D;
  objects_in_boxes2d->objects_vector.push_back(getObjectInBox(0, 0, 5, 5, "person", 0.99));
  objects_in_boxes2d->objects_vector.push_back(getObjectInBox(6, 6, 5, 5, "person", 0.99));

  ObjectsInBoxes3D::Ptr objects_in_boxes3d(new ObjectsInBoxes3D);
  std_msgs::Header header3D = createHeader(1, ros::Time(0, 1001), "camera_rgb_optical_frame");
  objects_in_boxes3d->header = header3D;

  ObjectsInBoxes3D::Ptr merged = Merger::merge(objects_in_boxes2d, objects_in_boxes3d);
  EXPECT_TRUE(merged->header == header2D);
  EXPECT_EQ(merged->objects_in_boxes.size(), 0);
}

TEST(UnitTestMerger, merge_Empty2D)
{
  ObjectsInBoxes::Ptr objects_in_boxes2d(new ObjectsInBoxes);
  std_msgs::Header header2D = createHeader(1, ros::Time(0, 1000), "camera_rgb_optical_frame");
  objects_in_boxes2d->header = header2D;

  ObjectsInBoxes3D::Ptr objects_in_boxes3d(new ObjectsInBoxes3D);
  std_msgs::Header header3D = createHeader(1, ros::Time(0, 1001), "camera_rgb_optical_frame");
  objects_in_boxes3d->header = header3D;
  objects_in_boxes3d->objects_in_boxes.push_back(getObjectInBox3D(0, 0, 100, 100, 1, 1, 1, 100, 100, 100));

  ObjectsInBoxes3D::Ptr merged = Merger::merge(objects_in_boxes2d, objects_in_boxes3d);
  EXPECT_TRUE(merged->header == header2D);
  EXPECT_EQ(merged->objects_in_boxes.size(), 0);
}

TEST(UnitTestMerger, merge_Normal)
{
  ObjectsInBoxes::Ptr objects_in_boxes2d(new ObjectsInBoxes);
  std_msgs::Header header2D = createHeader(1, ros::Time(0, 1000), "camera_rgb_optical_frame");
  objects_in_boxes2d->header = header2D;
  objects_in_boxes2d->objects_vector.push_back(getObjectInBox(0, 0, 100, 100, "person", 0.99));
  objects_in_boxes2d->objects_vector.push_back(getObjectInBox(101, 101, 100, 100, "person", 0.99));

  ObjectsInBoxes3D::Ptr objects_in_boxes3d(new ObjectsInBoxes3D);
  std_msgs::Header header3D = createHeader(1, ros::Time(0, 1001), "camera_rgb_optical_frame");
  objects_in_boxes3d->header = header3D;
  ObjectInBox3D person = getObjectInBox3D(0, 0, 100, 100, 1, 1, 1, 100, 100, 100);
  objects_in_boxes3d->objects_in_boxes.push_back(person);

  ObjectsInBoxes3D::Ptr merged = Merger::merge(objects_in_boxes2d, objects_in_boxes3d);
  EXPECT_TRUE(merged->header == header2D);
  EXPECT_EQ(merged->objects_in_boxes.size(), 1);
  EXPECT_TRUE(merged->objects_in_boxes[0] == person);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
