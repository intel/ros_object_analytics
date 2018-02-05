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

TEST(UnitTestObjectUtils, fill2DObjects_Empty)
{
  ObjectsInBoxes::Ptr objects(new ObjectsInBoxes);
  std::vector<Object2D> objects2d;
  ObjectUtils::fill2DObjects(objects, objects2d);
  EXPECT_EQ(objects2d.size(), 0);
}

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

TEST(UnitTestObjectUtils, fill3DObjects_Empty)
{
  ObjectsInBoxes3D::Ptr objects(new ObjectsInBoxes3D);
  std::vector<Object3D> objects3d;
  EXPECT_EQ(objects3d.size(), 0);
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

  pcl::PointCloud<PointXYZPixel>::Ptr cloudPixel(new pcl::PointCloud<PointXYZPixel>);

  std::vector<int> indices;
  for (auto i = 0; i < static_cast<int>(cloud->size()); i++)
  {
    indices.push_back(i);
  }
  ObjectUtils::copyPointCloud(cloud, indices, cloudPixel);

  PointXYZPixel x_min, x_max;
  ObjectUtils::getMinMaxPointsInX(cloudPixel, x_min, x_max);
  EXPECT_TRUE(x_min == getPointT(1.1, 2.2, 3.3));
  EXPECT_TRUE(x_max == getPointT(10.1, 8.2, 8.3));

  PointXYZPixel y_min, y_max;
  ObjectUtils::getMinMaxPointsInY(cloudPixel, y_min, y_max);
  EXPECT_TRUE(y_min == getPointT(2.1, 1.2, 2.3));
  EXPECT_TRUE(y_max == getPointT(9.1, 10.2, 9.3));

  PointXYZPixel z_min, z_max;
  ObjectUtils::getMinMaxPointsInZ(cloudPixel, z_min, z_max);
  EXPECT_TRUE(z_min == getPointT(3.1, 3.2, 1.3));
  EXPECT_TRUE(z_max == getPointT(8.1, 9.2, 10.3));
}

TEST(UnitTestObjectUtils, copyPointCloud_All)
{
  PointCloudT::Ptr cloud(new PointCloudT);
  readPointCloudFromPCD(std::string(RESOURCE_DIR) + "/copy.pcd", cloud);

  std::vector<int> indices;
  for (auto i = 0; i < static_cast<int>(cloud->size()); i++)
  {
    indices.push_back(i);
  }

  pcl::PointCloud<PointXYZPixel>::Ptr seg(new pcl::PointCloud<PointXYZPixel>);
  ObjectUtils::copyPointCloud(cloud, indices, seg);

  for (uint32_t y = 0; y < 5; ++y)
  {
    for (uint32_t x = 0; x < 5; ++x)
    {
      PointXYZPixel p = seg->points[y * 5 + x];
      EXPECT_TRUE(p.pixel_x == x);
      EXPECT_TRUE(p.pixel_y == y);
    }
  }
}

TEST(UnitTestObjectUtils, copyPointCloud_Empty)
{
  PointCloudT::Ptr cloud(new PointCloudT);
  readPointCloudFromPCD(std::string(RESOURCE_DIR) + "/copy.pcd", cloud);

  std::vector<int> indices;

  pcl::PointCloud<PointXYZPixel>::Ptr seg(new pcl::PointCloud<PointXYZPixel>);
  ObjectUtils::copyPointCloud(cloud, indices, seg);
  EXPECT_EQ(seg->size(), 0);
}

TEST(UnitTestObjectUtils, copyPointCloud_Diagonal)
{
  PointCloudT::Ptr cloud(new PointCloudT);
  readPointCloudFromPCD(std::string(RESOURCE_DIR) + "/copy.pcd", cloud);
  int i[] = { 0, 6, 12, 18, 24 };
  std::vector<int> indices(i, i + sizeof(i) / sizeof(uint32_t));

  pcl::PointCloud<PointXYZPixel>::Ptr seg(new pcl::PointCloud<PointXYZPixel>);
  ObjectUtils::copyPointCloud(cloud, indices, seg);

  for (uint32_t i = 0; i < 5; ++i)
  {
    EXPECT_EQ(seg->points[i].pixel_x, i);
    EXPECT_EQ(seg->points[i].pixel_y, i);
  }
}

TEST(UnitTestObjectUtils, getProjectedROI_NormalShapeNoEdgeOnImageBorder)
{
  PointCloudT::Ptr cloud(new PointCloudT);
  readPointCloudFromPCD(std::string(RESOURCE_DIR) + "/project.pcd", cloud);
  // NOLINTNEXTLINE
  int i[] = {
    15,                          // y_offset is 1
    23, 24, 25, 26, 27, 28,      // no edge this row
    32, 33, 34, 35, 36, 37, 38,  // width is 8 - 1 = 7
    42, 43, 44,                  // no edge this row
    51, 52, 53, 54,              // x_offset is 1
    62, 63, 64,                  // no edge this row
    72, 73, 74, 75, 76, 77, 78,  // width is 8 - 1 = 7
    86                           // height is 8 - 1 = 7
  };
  std::vector<int> indices(i, i + sizeof(i) / sizeof(uint32_t));

  pcl::PointCloud<PointXYZPixel>::Ptr seg(new pcl::PointCloud<PointXYZPixel>);
  ObjectUtils::copyPointCloud(cloud, indices, seg);
  sensor_msgs::RegionOfInterest roi;
  ObjectUtils::getProjectedROI(seg, roi);
  EXPECT_EQ(roi.x_offset, 1);
  EXPECT_EQ(roi.y_offset, 1);
  EXPECT_EQ(roi.width, 7);
  EXPECT_EQ(roi.height, 7);
}

TEST(UnitTestObjectUtils, getProjectedROI_NormalShapeAllEdgesOnImageBorder)
{
  PointCloudT::Ptr cloud(new PointCloudT);
  readPointCloudFromPCD(std::string(RESOURCE_DIR) + "/project.pcd", cloud);
  // NOLINTNEXTLINE
  int i[] = {
    5,                               // y_offset is 0
    23, 24, 25, 26, 27, 28,          // no edge this row
    32, 33, 34, 35, 36, 37, 38, 39,  // width is 9 - 0 = 9
    42, 43, 44,                      // no edge this row
    51, 52, 53, 54,                  // no edge this row
    60, 61, 62, 63, 64,              // x_offset is 0
    73, 74, 75, 76, 77, 78,          // no edge this row
    85, 86,                          // no edge this row
    96                               // height is 9 - 0 = 9
  };
  std::vector<int> indices(i, i + sizeof(i) / sizeof(uint32_t));

  pcl::PointCloud<PointXYZPixel>::Ptr seg(new pcl::PointCloud<PointXYZPixel>);
  ObjectUtils::copyPointCloud(cloud, indices, seg);
  sensor_msgs::RegionOfInterest roi;
  ObjectUtils::getProjectedROI(seg, roi);
  EXPECT_EQ(roi.x_offset, 0);
  EXPECT_EQ(roi.y_offset, 0);
  EXPECT_EQ(roi.width, 9);
  EXPECT_EQ(roi.height, 9);
}

TEST(UnitTestObjectUtils, getProjectedROI_ShapeIsFullOfImage)
{
  PointCloudT::Ptr cloud(new PointCloudT);
  readPointCloudFromPCD(std::string(RESOURCE_DIR) + "/project.pcd", cloud);
  std::vector<int> indices;
  for (int i = 0; i < 100; i++)
  {
    indices.push_back(i);
  }

  pcl::PointCloud<PointXYZPixel>::Ptr seg(new pcl::PointCloud<PointXYZPixel>);
  ObjectUtils::copyPointCloud(cloud, indices, seg);
  sensor_msgs::RegionOfInterest roi;
  ObjectUtils::getProjectedROI(seg, roi);
  EXPECT_EQ(roi.x_offset, 0);
  EXPECT_EQ(roi.y_offset, 0);
  EXPECT_EQ(roi.width, 9);
  EXPECT_EQ(roi.height, 9);
}

TEST(UnitTestObjectUtils, getProjectedROI_ShapeIsOneRow)
{
  PointCloudT::Ptr cloud(new PointCloudT);
  readPointCloudFromPCD(std::string(RESOURCE_DIR) + "/project.pcd", cloud);
  int i[] = { 23, 24, 25, 26, 27, 28 };
  std::vector<int> indices(i, i + sizeof(i) / sizeof(uint32_t));

  pcl::PointCloud<PointXYZPixel>::Ptr seg(new pcl::PointCloud<PointXYZPixel>);
  ObjectUtils::copyPointCloud(cloud, indices, seg);
  sensor_msgs::RegionOfInterest roi;
  ObjectUtils::getProjectedROI(seg, roi);
  EXPECT_EQ(roi.x_offset, 3);
  EXPECT_EQ(roi.y_offset, 2);
  EXPECT_EQ(roi.width, 5);
  EXPECT_EQ(roi.height, 0);
}

TEST(UnitTestObjectUtils, getProjectedROI_ShapeIsOneColumn)
{
  PointCloudT::Ptr cloud(new PointCloudT);
  readPointCloudFromPCD(std::string(RESOURCE_DIR) + "/project.pcd", cloud);
  int i[] = { 23, 33, 43, 53, 63, 73 };
  std::vector<int> indices(i, i + sizeof(i) / sizeof(uint32_t));

  pcl::PointCloud<PointXYZPixel>::Ptr seg(new pcl::PointCloud<PointXYZPixel>);
  ObjectUtils::copyPointCloud(cloud, indices, seg);
  sensor_msgs::RegionOfInterest roi;
  ObjectUtils::getProjectedROI(seg, roi);
  EXPECT_EQ(roi.x_offset, 3);
  EXPECT_EQ(roi.y_offset, 2);
  EXPECT_EQ(roi.width, 0);
  EXPECT_EQ(roi.height, 5);
}

TEST(UnitTestObjectUtils, getMatch_NoMatch)
{
  cv::Rect2d left(0, 0, 100, 100);
  cv::Rect2d right(101, 101, 100, 100);
  EXPECT_EQ(ObjectUtils::getMatch(left, right), 0);
}

TEST(UnitTestObjectUtils, getMatch_ExactMatchBetterThanBiggerMatch)
{
  cv::Rect2d origin(50, 50, 100, 100);
  cv::Rect2d exact(50, 50, 100, 100);
  cv::Rect2d bigger(40, 40, 110, 110);
  EXPECT_GT(ObjectUtils::getMatch(origin, exact), ObjectUtils::getMatch(origin, bigger));
}

TEST(UnitTestObjectUtils, getMatch_SameOverlapBetterDiviation)
{
  cv::Rect2d origin(0, 0, 100, 100);
  cv::Rect2d bigDiviation(0, 0, 50, 50);
  cv::Rect2d smallDiviation(20, 20, 70, 70);
  EXPECT_GT(ObjectUtils::getMatch(origin, smallDiviation), ObjectUtils::getMatch(origin, bigDiviation));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
