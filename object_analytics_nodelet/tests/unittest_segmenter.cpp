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
#include <cassert>
#include <vector>
#include <gtest/gtest.h>
#include <pcl_conversions/pcl_conversions.h>
#include "object_analytics_nodelet/segmenter/segmenter.h"
#include "object_analytics_nodelet/segmenter/algorithm.h"
#include "object_analytics_nodelet/segmenter/algorithm_provider.h"
#include "tests/unittest_util.h"

using object_analytics_nodelet::segmenter::Algorithm;
using object_analytics_nodelet::segmenter::AlgorithmProvider;
using object_analytics_nodelet::segmenter::Segmenter;

class Algo : public Algorithm
{
public:
  Algo() = default;
  ~Algo() = default;

  void segment(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud,
               pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud_segment, std::vector<pcl::PointIndices>& cluster_indices)
  {
    pcl::PointIndices indices;
    for (int i = 0; i < 15; i++)
    {
      indices.indices.push_back(i);
    }
    cluster_indices.push_back(indices);
    pcl::copyPointCloud(*cloud, *cloud_segment);
  }
};

class AlgoProvider : public AlgorithmProvider
{
public:
  virtual std::shared_ptr<Algorithm> get()
  {
    return algo_;
  }

  AlgoProvider() : algo_(std::make_shared<Algo>())
  {
  }

  ~AlgoProvider() = default;

private:
  std::shared_ptr<Algorithm> algo_;
};

TEST(UnitTestSegmenter, segmenter)
{
  PointCloudT::Ptr cloud(new PointCloudT);
  readPointCloudFromPCD(std::string(RESOURCE_DIR) + "/segment.pcd", cloud);

  sensor_msgs::PointCloud2::Ptr cloudMsg(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*cloud, *cloudMsg);

  std::unique_ptr<Segmenter> impl;
  impl.reset(new Segmenter(std::unique_ptr<AlgoProvider>(new AlgoProvider())));

  boost::shared_ptr<ObjectsInBoxes3D> obj3d(new ObjectsInBoxes3D);
  impl->segment(cloudMsg, obj3d);

  EXPECT_EQ(1, obj3d->objects_in_boxes.size());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
