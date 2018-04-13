#!/usr/bin/env python
"""
 Copyright (c) 2017 Intel Corporation

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
"""
import cv2
import rospy
import message_filters
from copy import deepcopy
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import RegionOfInterest
from object_msgs.msg import ObjectsInBoxes
from object_analytics_msgs.msg import TrackedObjects
from object_analytics_msgs.msg import ObjectsInBoxes3D

"""@package image_publisher
Draw detection as well as tracking results on original RGB image, and republish so that could be shown on RViz image
display
"""


class Roi(object):
    """Wrapper of sensor_msgs.msg.RegionOfInterest for hashing

    when roi is used to be as dict key, it's required to have its own hash implementation which needs to implement two
    special methods __hash__ and __eq__
    """

    def __init__(self, roi):
        """Construct a Roi from sensor_msgs.msg.RegionOfInterest instance

        Args:
            roi (RegionOfInterest): Roi of a detected object or tracked object
        """
        self._hash = hash((roi.x_offset, roi.y_offset, roi.width, roi.height))

    def __hash__(self):
        return self._hash

    def __eq__(self, other):
        return hash(self) == hash(other)


class Measure(object):
    """class to measure FPS and latency"""

    def __init__(self, name, topic, msg_type):
        """Init subscriber of specific topic

        Args:
            name  (str):  name
            topic (str):  topic name
        """
        self._name = name
        self._fps = 0
        self._latency = 0
        self._last = None
        self._count = 0
        rospy.Subscriber(topic, msg_type, self._callback)

    def _callback(self, msg):
        """subscriber callback"""
        now = rospy.get_rostime()
        self._latency = (now - msg.header.stamp).to_sec()

        self._count = self._count + 1
        if self._last is None:
            self._last = now
        elif now - self._last > rospy.Duration(1.0):
            self._fps = self._count / (now - self._last).to_sec()
            self._count = 0
            self._last = now

    def __str__(self):
        return "{}:fps={:.2f}Hz, latency={:.2f}sec".format(self._name, self._fps, self._latency)


class ObjectItem(object):
    """An ObjectItem represents a merged result of detection and tracking who have the same roi.x_offset

    The main usage of this class is to draw(attach) rectangle and text with the original image
    """

    RED = (255, 0, 0)
    YELLOW = (255, 255, 0)
    GREEN = (0, 255, 0)
    FONT = cv2.FONT_HERSHEY_SIMPLEX
    FONT_SIZE = 0.5

    def __init__(self, cv_image, roi, track_id, detected_object):
        """Build an instance from cv Mat, RegionOfInterest, tracking id and detected object

        Args:
            cv_image (cv2.Mat):         Mat convereted from rgb image
            roi (RegionOfInterest):     Region of interest
            track_id (int):             Tracking id
            detected_object (Object):   Instance of object_msgs.msg.Object
        """
        self._cv_image = cv_image
        self._roi = roi
        self._track_id = track_id
        self._object = detected_object
        self._top_left = (roi.x_offset, roi.y_offset)
        self._down_right = (roi.x_offset + roi.width, roi.y_offset + roi.height)

    def draw(self):
        """Draw rectangle, name, tracking id and roi onto cv Mat.
        """
        self._draw_rectangle()
        self._draw_roi_text()
        self._draw_name_id()

    def _draw_rectangle(self):
        """Draw rectangle same size and position as roi.
        """
        cv2.rectangle(self._cv_image, self._top_left, self._down_right, self.YELLOW, 0)

    def _draw_roi_text(self):
        """Draw roi text on the top left position.
        """
        text = "ROI[{},{},{},{}]".format(self._roi.x_offset, self._roi.y_offset, self._roi.width, self._roi.height)
        cv2.putText(self._cv_image, text, self._top_left, self.FONT, self.FONT_SIZE, self.GREEN, 0, cv2.LINE_AA)

    def _draw_name_id(self):
        """Draw object name and tracking id together in the middle left of the rectangle
        """
        text = "{} #{}".format(self._object.object_name, self._track_id)
        pos = (self._roi.x_offset, self._roi.y_offset + self._roi.height / 2)
        cv2.putText(self._cv_image, text, pos, self.FONT, self.FONT_SIZE, self.GREEN, 1, cv2.LINE_AA)


class SynchronizedSubscriber(object):
    """Time synchronizer wrapper for tracking, detection and original rgb image.

    Create a TimeSynchronizer to synchronize rgb image, detection and tracking topics published by object analytics,
    draw approprate rectangle and text onto original rgb image and republish the updated rgb image.
    """

    PUB_TOPIC = 'tracking_rviz'

    def __init__(self):
        """Create TimeSynchronizer to listen on rgb image, detection result and tracking results
        """
        self._bridge = CvBridge()
        self._pub = rospy.Publisher(self.PUB_TOPIC, Image, queue_size=10)
        image_sub = message_filters.Subscriber('/object_analytics/rgb', Image)
        detection_sub = message_filters.Subscriber('/object_analytics/detection', ObjectsInBoxes)
        tracking_sub = message_filters.Subscriber('/object_analytics/tracking', TrackedObjects)

        ts = message_filters.TimeSynchronizer([image_sub, detection_sub, tracking_sub], 100)
        ts.registerCallback(self._callback)

        self._measures = [Measure('Detection', '/object_analytics/detection', ObjectsInBoxes),
                          Measure('Localization', '/object_analytics/localization', ObjectsInBoxes3D),
                          Measure('Tracking', '/object_analytics/tracking', TrackedObjects)]

    def _callback(self, image, detected_objects, tracked_objects):
        """TimeSynchronizer callback.

        Args:
            image (sensor_msgs.msg.Image):                              Rgb image of cuurent processing frame
            detected_objects (object_msgs.msg.ObjectsInBoxes):          Detection results of current frame
            tracked_objects (object_analytics_msgs.msg.TrackedObjects): Tracking results of current frame
        """
        try:
            header = image.header
            cv_image = self._bridge.imgmsg_to_cv2(image, "bgr8")

            objects = self._merge_results(cv_image, detected_objects, tracked_objects)
            for obj in objects:
                obj.draw()
            self._draw_measures(cv_image)

            msg = self._bridge.cv2_to_imgmsg(cv_image, "bgr8")
            msg.header = header
            self._pub.publish(msg)

            rospy.loginfo(" ".join((str(m) for m in self._measures)))

        except CvBridgeError as e:
            rospy.logerr(str(e))
        except Exception as e:
            rospy.logerr(str(e))

    def _draw_measures(self, cv_image):
        """Draw measure result on the left up"""
        text = "\n".join(" ".join((str(m) for m in self._measures)))
        for idx, m in enumerate(self._measures):
            cv2.putText(cv_image, str(m), (2, 15+idx*15), ObjectItem.FONT,
                        ObjectItem.FONT_SIZE, ObjectItem.RED, 1, cv2.LINE_AA)

    @staticmethod
    def _merge_results(cv_image, detected_objects, tracked_objects):
        """Merge detection and tracking result which have the same roi, roi should must be included in tracking results.

        Args:
            cv_image (cv2.Mat):                                         Rgb image of cuurent processing frame
            detected_objects (object_msgs.msg.ObjectsInBoxes):          Detection results of current frame
            tracked_objects (object_analytics_msgs.msg.TrackedObjects): Tracking results of current frame

        Returns:
            ObjectItem list: list of ObjectItem in which each item is merged from detction and tracking
        """
        tracking_map = {}
        for obj in tracked_objects.tracked_objects:
            tracking_map[Roi(obj.roi)] = obj

        merged = []
        for obj in detected_objects.objects_vector:
            key = Roi(obj.roi)
            if key not in tracking_map:
                continue
            merged.append(ObjectItem(cv_image, obj.roi, tracking_map[key].id, obj.object))

        return merged


if __name__ == "__main__":
    rospy.init_node('object_analytics_visualization2d')
    SynchronizedSubscriber()
    rospy.spin()
