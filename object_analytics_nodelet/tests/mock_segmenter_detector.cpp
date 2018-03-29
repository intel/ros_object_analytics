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
#include <ros/ros.h>
#include <object_msgs/ObjectsInBoxes.h>
#include <object_analytics_msgs/ObjectsInBoxes3D.h>
#include "tests/unittest_util.h"

using object_msgs::ObjectsInBoxes;
using object_analytics_msgs::ObjectsInBoxes3D;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "TestSegmenterDetector");
  ros::NodeHandle nh;
  ros::Publisher pubDetection = nh.advertise<ObjectsInBoxes>("detection", 1);
  ros::Publisher pubSegmentation = nh.advertise<ObjectsInBoxes3D>("segmentation", 1);
  ros::Rate rate(1);

  ObjectsInBoxes obj;
  obj.objects_vector.push_back(getObjectInBox(10, 10, 100, 100, "person", 0.99));

  ObjectsInBoxes3D obj3d;
  obj3d.objects_in_boxes.push_back(getObjectInBox3D(5, 5, 100, 100, 1, 1, 1, 1.1, 1.1, 1.1));

  while (ros::ok())
  {
    pubDetection.publish(obj);
    pubSegmentation.publish(obj3d);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
