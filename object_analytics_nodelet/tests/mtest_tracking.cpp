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
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <object_msgs/ObjectsInBoxes.h>
#include <object_analytics_msgs/TrackedObjects.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using SubscribeImg = message_filters::Subscriber<sensor_msgs::Image>;
using SubscribeTracks = message_filters::Subscriber<object_analytics_msgs::TrackedObjects>;
using SyncTracking = message_filters::TimeSynchronizer<sensor_msgs::Image, object_analytics_msgs::TrackedObjects>;

static const std::string kCvWindowName = "Object Analytics View";
static bool received_tracks;
#ifdef MTEST_TRACKING_ENABLE_VIEW
static object_msgs::ObjectsInBoxesConstPtr dobjs = nullptr;
static ros::Duration dlatency;
static int32_t dfps;
#endif

static void tracking_cb(const sensor_msgs::ImageConstPtr& rgb,
                        const object_analytics_msgs::TrackedObjectsConstPtr& tracks)
{
#ifndef MTEST_TRACKING_ENABLE_VIEW
  received_tracks = true;
#else
  static ros::Time tracking_stamp = ros::Time();
  static uint64_t tracking_count = 0;
  char buf[256];
  cv::Mat mat = cv_bridge::toCvCopy(rgb, "bgr8")->image;
  if (!tracking_count)
  {
    tracking_stamp = ros::Time::now();
  }
  else
  {
    ros::Duration d = ros::Time::now() - tracking_stamp;
    ros::Duration l = ros::Time::now() - tracks->header.stamp;
    int32_t tfps = 1000000000ll / (d.toNSec() / tracking_count);
    snprintf(buf, sizeof(buf), "tracking FPS %d latency %d.%09d", tfps, l.sec, l.nsec);
    cv::putText(mat, buf, cv::Point(0, 12), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 0, 0));
    snprintf(buf, sizeof(buf), "detection FPS %d latency %d.%09d", dfps, dlatency.sec, dlatency.nsec);
    cv::putText(mat, buf, cv::Point(0, 24), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 0, 0));
  }
  tracking_count++;
  for (auto t : tracks->tracked_objects)
  {
    sensor_msgs::RegionOfInterest troi = t.roi;
    cv::Rect2d r = cv::Rect2d(troi.x_offset, troi.y_offset, troi.width, troi.height);
    cv::Scalar green = cv::Scalar(0, 255, 0);
    cv::rectangle(mat, r, green);
    snprintf(buf, sizeof(buf), "ID [%d]", t.id);
    cv::putText(mat, buf, cv::Point(r.x, r.y + 32), cv::FONT_HERSHEY_SIMPLEX, 1.0, green);
  }
  if (dobjs)
  {
    for (auto d : dobjs->objects_vector)
    {
      object_msgs::Object dobj = d.object;
      sensor_msgs::RegionOfInterest droi = d.roi;
      cv::Rect2d r = cv::Rect2d(droi.x_offset, droi.y_offset, droi.width, droi.height);
      std::string n = std::string(dobj.object_name.data());
      cv::Scalar red = cv::Scalar(0, 0, 255);
      cv::rectangle(mat, r, red);
      snprintf(buf, sizeof(buf), "%s [%.0f%%]", n.c_str(), dobj.probability * 100);
      cv::putText(mat, buf, cv::Point(r.x, r.y + 48), cv::FONT_HERSHEY_PLAIN, 1.0, red);
    }
  }
  imshow(kCvWindowName, mat);
  cv::waitKey(1);
#endif
}

static void detection_cb(const object_msgs::ObjectsInBoxesConstPtr& objs)
{
#ifdef MTEST_TRACKING_ENABLE_VIEW
  static ros::Time detection_stamp = ros::Time();
  static uint64_t detection_count = 0;
  if (!detection_count)
  {
    detection_stamp = ros::Time::now();
  }
  else
  {
    ros::Duration d = ros::Time::now() - detection_stamp;
    dfps = 1000000000ll / (d.toNSec() / detection_count);
    dlatency = ros::Time::now() - objs->header.stamp;
  }
  detection_count++;

  dobjs = objs;
#endif
}

TEST(TestTracking, testTrackedObjects)
{
#ifndef MTEST_TRACKING_ENABLE_VIEW
  ros::Duration(10).sleep();
  ros::spinOnce();
// todo: uncomment this testpoint once CI environment ready
// EXPECT_TRUE(received_tracks);
#else
  cv::namedWindow(kCvWindowName);
  ros::spin();
#endif
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  received_tracks = false;

  ros::init(argc, argv, "mtest_tracking");
  ros::NodeHandle nh;
  SubscribeImg sub_rgb(nh, "/object_analytics/rgb", 30);
  SubscribeTracks sub_track(nh, "/object_analytics/tracking", 30);

  SyncTracking sync_track(sub_rgb, sub_track, 30);
  sync_track.registerCallback(tracking_cb);
  ros::Subscriber sub_obj = nh.subscribe("/object_analytics/detection", 10, detection_cb);
  return RUN_ALL_TESTS();
}
