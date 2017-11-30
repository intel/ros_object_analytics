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

#ifndef OBJECT_ANALYTICS_NODELET_MODEL_PROJECTOR_IMPL_H
#define OBJECT_ANALYTICS_NODELET_MODEL_PROJECTOR_IMPL_H

#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>

#include "object_analytics_nodelet/model/projector.h"

namespace object_analytics_nodelet
{
namespace model
{
/** @class ProjectorImpl
 * Implementation of Projector interface. Leverage image_geometry package to do the projection.
 */
class ProjectorImpl : public Projector
{
public:
  /**
   * Static method. Get singleton instance.
   *
   * @return Shared pointer pointing to the singleton instance
   */
  static std::shared_ptr<ProjectorImpl> instance();

  /** Constructor */
  ProjectorImpl();

  /** Default destructor */
  ~ProjectorImpl() = default;

  /**
   * Inherit from Projector. Interface to project object in 3d space into pixel space.
   *
   * @param[in]   point   3d point to be projected on pixel space
   * @param[out]  x       X in pixel
   * @param[out]  y       Y in pixel
   */
  void project3dToPixel(const geometry_msgs::Point32& point, int& x, int& y);

private:
  static std::shared_ptr<ProjectorImpl> instance_;
  image_geometry::PinholeCameraModel model_;
};
}  // namespace model
}  // namespace object_analytics_nodelet
#endif  // OBJECT_ANALYTICS_NODELET_MODEL_PROJECTOR_IMPL_H
