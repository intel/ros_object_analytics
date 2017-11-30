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

#ifndef OBJECT_ANALYTICS_NODELET_SEGMENTER_ALGORITHM_PROVIDER_H
#define OBJECT_ANALYTICS_NODELET_SEGMENTER_ALGORITHM_PROVIDER_H

#include <map>
#include <string>

#include <dynamic_reconfigure/server.h>

#include "object_analytics_nodelet/SegmentationAlgorithmsConfig.h"
#include "object_analytics_nodelet/segmenter/algorithm.h"

namespace object_analytics_nodelet
{
namespace segmenter
{
using object_analytics_nodelet::SegmentationAlgorithmsConfig;
using object_analytics_nodelet::segmenter::Algorithm;

/** @class AlorithmProvider
 * Segmentation algorithm factory class
 */
class AlgorithmProvider
{
public:
  /**
   * Constructor. Initialize algorithm map.
   *
   * @param[in] nh Ros NodeHandle
   */
  explicit AlgorithmProvider(ros::NodeHandle& nh);

  /**
   * Get current selected algorithm instance
   *
   * @return Pointer to current slected algorithm instance
   */
  std::shared_ptr<Algorithm> get();

private:
  /**
   * Callback method. Called everytime SegmentationAlgorithmsConfig is changed, member variable conf_ will be updated
   * accordingly.
   *
   * @param[in] config  Changed SegmentationAlgorithmsConfig instance
   * @param[in] level   Not used
   */
  void cbConfig(SegmentationAlgorithmsConfig& config, uint32_t level);

  static const std::string DEFAULT;

  std::map<std::string, std::shared_ptr<Algorithm>> algorithms_;
  boost::shared_ptr<dynamic_reconfigure::Server<SegmentationAlgorithmsConfig>> conf_srv_;
  SegmentationAlgorithmsConfig conf_;
};
}  // namespace segmenter
}  // namespace object_analytics_nodelet
#endif  // OBJECT_ANALYTICS_NODELET_SEGMENTER_ALGORITHM_PROVIDER_H
