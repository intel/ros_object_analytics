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

#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>

#include "object_analytics_nodelet/model/object_utils.h"

#include "tests/unittest_util.h"

TEST(UnitTestObjectUtils, fill2DObjects_NormalCase)
{
  ObjectsInBoxes::Ptr objects(new ObjectsInBoxes);
  ObjectInBox first = getObjectInBox(0, 0, 100, 100, "person", 0.9f);
  objects->objects_vector.push_back(first);
  ObjectInBox second = getObjectInBox(100, 100, 50, 50, "chair", 0.8f);
  objects->objects_vector.push_back(second);
  ObjectInBox third = getObjectInBox(320, 480, 1, 1, "key", 0.1f);
  objects->objects_vector.push_back(third);
  std::vector<Object2D> objects2d;
  ObjectUtils::fill2DObjects(objects, objects2d);
  EXPECT_EQ(objects2d.size(), 3);
  EXPECT_TRUE(first == objects2d[0]);
  EXPECT_TRUE(second == objects2d[1]);
  EXPECT_TRUE(third == objects2d[2]);
}

TEST(UnitTestObjectUtils, fill3DObjects_NormalCase)
{
  ObjectsInBoxes3D::Ptr objects(new ObjectsInBoxes3D);
  ObjectInBox3D first = getObjectInBox3D(1, 1, 100, 100, 1, 2, 3, 4, 5, 6);
  objects->objects_in_boxes.push_back(first);
  ObjectInBox3D second = getObjectInBox3D(100, 100, 200, 200, 7, 8, 9, 10, 11, 12);
  objects->objects_in_boxes.push_back(second);
  ObjectInBox3D third = getObjectInBox3D(320, 480, 1, 1, 13, 14, 15, 16, 17, 18);
  objects->objects_in_boxes.push_back(third);
  std::vector<Object3D> objects3d;
  ObjectUtils::fill3DObjects(objects, objects3d);
  EXPECT_EQ(objects3d.size(), 3);
  EXPECT_TRUE(first == objects3d[0]);
  EXPECT_TRUE(second == objects3d[1]);
  EXPECT_TRUE(third == objects3d[2]);
}

TEST(UnitTestObjectUtils, findMaxIntersectionRelationships_NormalCase)
{
  std::vector<Object3D> objects3d;
  std::vector<Object2D> objects2d;
  std::vector<std::pair<Object2D, Object3D>> relations;
  // build 3d objects
  Object3D first3d = Object3D(getObjectInBox3D(1, 1, 100, 100, 1, 2, 3, 4, 5, 6));
  Object3D second3d = Object3D(getObjectInBox3D(1, 102, 100, 100, 1, 2, 3, 4, 5, 6));
  Object3D third3d = Object3D(getObjectInBox3D(50, 50, 100, 100, 1, 2, 3, 4, 5, 6));
  Object3D forth3d = Object3D(getObjectInBox3D(200, 0, 30, 40, 1, 2, 3, 4, 5, 6));
  Object3D fifth3d = Object3D(getObjectInBox3D(49, 49, 60, 60, 1, 2, 3, 4, 5, 6));
  objects3d.push_back(first3d);
  objects3d.push_back(second3d);
  objects3d.push_back(third3d);
  objects3d.push_back(forth3d);
  objects3d.push_back(fifth3d);
  // build 2d objects
  Object2D first2d = Object2D(getObjectInBox(0, 0, 100, 100, "person", 0.8f));
  Object2D second2d = Object2D(getObjectInBox(0, 101, 100, 100, "dog", 0.9f));
  Object2D third2d = Object2D(getObjectInBox(49, 49, 60, 60, "chair", 0.3f));
  Object2D forth2d = Object2D(getObjectInBox(200, 200, 30, 40, "computer", 0.8f));
  objects2d.push_back(first2d);
  objects2d.push_back(second2d);
  objects2d.push_back(third2d);
  objects2d.push_back(forth2d);
  ObjectUtils::findMaxIntersectionRelationships(objects2d, objects3d, relations);
  EXPECT_EQ(relations.size(), 3);
  std::pair<Object2D, Object3D> first = relations[0];
  EXPECT_TRUE(first.first == first2d);
  EXPECT_TRUE(first.second == first3d);
  std::pair<Object2D, Object3D> second = relations[1];
  EXPECT_TRUE(second.first == second2d);
  EXPECT_TRUE(second.second == second3d);
  std::pair<Object2D, Object3D> third = relations[2];
  EXPECT_TRUE(third.first == third2d);
  EXPECT_TRUE(third.second == fifth3d);
}

TEST(UnitTestObjectUtils, getMinMaxPointsInXYZ)
{
  PointCloudT::Ptr cloud(new PointCloudT);
  readPointCloudFromPCD(std::string(RESOURCE_DIR) + "/object3d.pcd", cloud);

  PointT x_min, x_max;
  ObjectUtils::getMinMaxPointsInX(cloud, x_min, x_max);
  EXPECT_TRUE(x_min == getPointT(1.1, 2.2, 3.3));
  EXPECT_TRUE(x_max == getPointT(10.1, 8.2, 8.3));

  PointT y_min, y_max;
  ObjectUtils::getMinMaxPointsInY(cloud, y_min, y_max);
  EXPECT_TRUE(y_min == getPointT(2.1, 1.2, 2.3));
  EXPECT_TRUE(y_max == getPointT(9.1, 10.2, 9.3));

  PointT z_min, z_max;
  ObjectUtils::getMinMaxPointsInZ(cloud, z_min, z_max);
  EXPECT_TRUE(z_min == getPointT(3.1, 3.2, 1.3));
  EXPECT_TRUE(z_max == getPointT(8.1, 9.2, 10.3));
}

TEST(UnitTestObjectUtils, intersectionArea)
{
  Object2D obj1(getObjectInBox(0, 0, 100, 100, "ppl", 1));

  cv::Rect2d base1(0, 0, 100, 100);
  cv::Rect2d match1(1, 1, 100, 100);
  cv::Rect2d not_match1(0, 0, 1000, 1000);
  EXPECT_TRUE(ObjectUtils::getMatch(base1, match1) > ObjectUtils::getMatch(base1, not_match1));

  cv::Rect2d base2(50, 50, 100, 100);
  cv::Rect2d match2(30, 30, 130, 130);
  cv::Rect2d not_match2(40, 40, 140, 140);
  EXPECT_TRUE(ObjectUtils::getMatch(base2, match2) > ObjectUtils::getMatch(base2, not_match2));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
