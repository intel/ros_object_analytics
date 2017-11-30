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

#ifndef OBJECT_ANALYTICS_NODELET_MERGER_MERGER_NODELET_H
#define OBJECT_ANALYTICS_NODELET_MERGER_MERGER_NODELET_H

#include <nodelet/nodelet.h>

#include "object_analytics_nodelet/merger/merger.h"

namespace object_analytics_nodelet
{
namespace merger
{
/** @class MergerNodelet
 * Merger nodelet, merger implementation holder.
 */
class MergerNodelet : public nodelet::Nodelet
{
public:
  /** Default destructor */
  ~MergerNodelet() = default;

private:
  /** Inherit from Nodelet class. Initialize Merger instance. */
  virtual void onInit();

  std::unique_ptr<Merger> impl_;
};
}  // namespace merger
}  // namespace object_analytics_nodelet
#endif  // OBJECT_ANALYTICS_NODELET_MERGER_MERGER_NODELET_H
