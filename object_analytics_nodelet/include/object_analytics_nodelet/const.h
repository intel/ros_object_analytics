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

#pragma once
#ifndef OBJECT_ANALYTICS_NODELET_CONST_H
#define OBJECT_ANALYTICS_NODELET_CONST_H

#include <string>

namespace object_analytics_nodelet
{
/** @class Const
 * A class contains global constatnts definition.
 */
class Const
{
public:
  static const std::string kTopicRegisteredPC2; /**< Topic name of splitter nodelet's input message */
  static const std::string kTopicPC2;           /**< Topic name of segmenter nodelet's input message */
  static const std::string kTopicRgb;           /**< Topic name of 2d detection's input message */
  static const std::string kTopicSegmentation;  /**< Topic name of segmenter nodelet's output message */
  static const std::string kTopicDetection;     /**< Topic name of 2d detection's output message */
  static const std::string kTopicLocalization;  /**< Topic name of merger nodelet's output message */
  static const std::string kTopicTracking;      /**< Topic name of tracker nodelet's output message */
};
}  // namespace object_analytics_nodelet
#endif  // OBJECT_ANALYTICS_NODELET_CONST_H
