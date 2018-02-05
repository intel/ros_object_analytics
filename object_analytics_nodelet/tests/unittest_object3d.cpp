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
#include <cassert>
#include <string>
#include <vector>
#include <utility>

#include <gtest/gtest.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "object_analytics_nodelet/model/object3d.h"

#include "tests/unittest_util.h"

TEST(UnitTestObject3D, constructor_TwoParameters)
{
  PointCloudT::Ptr cloud(new PointCloudT);
  readPointCloudFromPCD(std::string(RESOURCE_DIR) + "/object3d.pcd", cloud);
  std::vector<int> indices;

  for (unsigned int i = 0; i < cloud->points.size(); i++)
  {
    indices.push_back(i);
  }

  Object3D obj3(cloud, indices);
  EXPECT_TRUE(obj3.getMin() == getPoint32(1.1, 1.2, 1.3));
  EXPECT_TRUE(obj3.getMax() == getPoint32(10.1, 10.2, 10.3));
  EXPECT_TRUE(obj3.getRoi() == getRoi(0, 0, 4, 1));
}

TEST(UnitTestObject3D, constructor_OneParameter)
{
  ObjectInBox3D objectInBox3D = getObjectInBox3D(0, 0, 100, 100, 20, 30, 40, 200, 300, 400);
  Object3D obj3(objectInBox3D);
  EXPECT_TRUE(obj3.getMin() == getPoint32(20, 30, 40));
  EXPECT_TRUE(obj3.getMax() == getPoint32(200, 300, 400));
  EXPECT_TRUE(obj3.getRoi() == getRoi(0, 0, 100, 100));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
