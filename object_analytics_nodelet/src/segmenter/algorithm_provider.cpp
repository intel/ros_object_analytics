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

#include <list>
#include <string>

#include <ros/ros.h>

#include "object_analytics_nodelet/SegmentationAlgorithmsConfig.h"
#include "object_analytics_nodelet/segmenter/organized_multi_plane_segmenter.h"
#include "object_analytics_nodelet/segmenter/algorithm_provider.h"

using object_analytics_nodelet::segmenter::OrganizedMultiPlaneSegmenter;

namespace object_analytics_nodelet
{
namespace segmenter
{
const std::string AlgorithmProvider::DEFAULT = "OrganizedMultiPlaneSegmentation";

AlgorithmProvider::AlgorithmProvider(ros::NodeHandle& nh)
{
  conf_srv_ = boost::make_shared<dynamic_reconfigure::Server<SegmentationAlgorithmsConfig>>(nh);
  conf_srv_->getConfigDefault(conf_);

  conf_srv_->setCallback(boost::bind(&AlgorithmProvider::cbConfig, this, _1, _2));

  algorithms_["OrganizedMultiPlaneSegmentation"] =
      std::static_pointer_cast<Algorithm>(std::make_shared<OrganizedMultiPlaneSegmenter>(nh));
}

std::shared_ptr<Algorithm> AlgorithmProvider::get()
{
  std::string name = conf_.algorithm;
  try
  {
    std::shared_ptr<Algorithm> algo = algorithms_.at(name);
    return algo;
  }
  catch (const std::out_of_range& e)
  {
    ROS_WARN_STREAM("Algorithm named " << name << " doesn't exist, use " << AlgorithmProvider::DEFAULT << " instead");
    return algorithms_.at(DEFAULT);
  }
}

void AlgorithmProvider::cbConfig(SegmentationAlgorithmsConfig& config, uint32_t level)
{
  conf_ = config;
}
}  // namespace segmenter
}  // namespace object_analytics_nodelet
