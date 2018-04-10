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

import rospy
import message_filters
from copy import deepcopy
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point32
from sensor_msgs.msg import RegionOfInterest
from object_msgs.msg import ObjectsInBoxes
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from object_analytics_msgs.msg import TrackedObjects
from object_analytics_msgs.msg import ObjectsInBoxes3D


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


class ObjectItem(object):
    """An ObjectItem represents a merged result of localization, detection and tracking who have the same roi.

    The main usage of this class is to build Marker based on the merged information.
    """

    YELLOW = ColorRGBA(1.0, 1.0, 0.0, 1.0)
    GREEN = ColorRGBA(0.0, 1.0, 0.0, 1.0)
    POSE = Pose()

    def __init__(self, header, roi, track_id, detected_object, min, max):
        """Build an instance from message header, region of interest, tracking id, detected object and 3d min max.

        Args:
            header (std_msgs.msg.Header):      Message header
            roi (RegionOfInterest):            Region of interest
            track_id (int):                    Tracking id
            detected_object (Object):          Instance of object_msgs.msg.Object
            min (geometry_msgs.msg.Point32):   Min position in 3d space
            max (geometry_msgs.msg.Point32):   Max position in 3d space
        """
        self._header = header
        self._roi = roi
        self._track_id = track_id
        self._object = detected_object

        self._p1 = deepcopy(min)

        self._p2 = deepcopy(self._p1)
        self._p2.x = max.x

        self._p3 = deepcopy(self._p2)
        self._p3.z = max.z

        self._p4 = deepcopy(self._p3)
        self._p4.x = min.x

        self._p5 = deepcopy(min)
        self._p5.y = max.y

        self._p6 = deepcopy(self._p5)
        self._p6.x = max.x

        self._p7 = deepcopy(self._p6)
        self._p7.z = max.z

        self._p8 = deepcopy(self._p7)
        self._p8.x = min.x

    def linelist(self):
        """Build line list marker from 8 points in 3d space
        """
        line_list = Marker()
        line_list.header = self._header
        line_list.type = Marker.LINE_LIST
        line_list.action = Marker.ADD
        line_list.scale.x = 0.005
        line_list.color = self.YELLOW
        line_list.pose = deepcopy(self.POSE)

        line_list.points.extend((self._p1, self._p2))
        line_list.points.extend((self._p2, self._p3))
        line_list.points.extend((self._p3, self._p4))
        line_list.points.extend((self._p4, self._p1))
        line_list.points.extend((self._p5, self._p6))
        line_list.points.extend((self._p6, self._p7))
        line_list.points.extend((self._p7, self._p8))
        line_list.points.extend((self._p8, self._p5))
        line_list.points.extend((self._p1, self._p5))
        line_list.points.extend((self._p2, self._p6))
        line_list.points.extend((self._p3, self._p7))
        line_list.points.extend((self._p4, self._p8))

        return line_list

    def min_text(self):
        """Build text marker to hold min text
        """
        text = Marker()
        text.header = self._header
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.scale.z = 0.05
        text.color = self.GREEN
        text.pose = deepcopy(self.POSE)
        text.pose.position = deepcopy(self._p1)
        text.text = "Min[{:.2f} {:.2f} {:.2f}]".format(self._p1.x, self._p1.y, self._p1.z)
        return text

    def max_text(self):
        """Build text marker to hold max text
        """
        text = Marker()
        text.header = self._header
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.scale.z = 0.05
        text.color = self.GREEN
        text.pose = deepcopy(self.POSE)
        text.pose.position = deepcopy(self._p7)
        text.text = "Max[{:.2f} {:.2f} {:.2f}]".format(self._p7.x, self._p7.y, self._p7.z)
        return text

    def name_id_text(self):
        """Build text marker to hold name and tracking id.
        """
        text = Marker()
        text.header = self._header
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.scale.z = 0.05
        text.color = self.GREEN
        text.pose = deepcopy(self.POSE)
        text.pose.position.x = self._p1.x
        text.pose.position.y = (self._p1.y + self._p5.y) / 2
        text.pose.position.z = self._p1.z
        text.text = "{} #{}".format(self._object.object_name, self._track_id)
        return text


class SynchronizedSubscriber(object):
    """Time synchronizer wrapper for localization"""

    TOPIC = 'localization_rviz'

    def __init__(self):
        """Create TimeSynchronizer to listen on Localization, detection result and tracking results
        """
        self._pub = rospy.Publisher(self.TOPIC, MarkerArray, queue_size=10)
        loc_sub = message_filters.Subscriber('/object_analytics/localization', ObjectsInBoxes3D)
        det_sub = message_filters.Subscriber('/movidius_ncs_stream/detected_objects', ObjectsInBoxes)
        tra_sub = message_filters.Subscriber('/object_analytics/tracking', TrackedObjects)

        ts = message_filters.TimeSynchronizer([det_sub, loc_sub, tra_sub], 10)
        ts.registerCallback(self._callback)

    def _callback(self, detected_objects, localized_objects, tracked_objects):
        """TimeSynchronizer callback.

        Args:
            detected_objects (object_msgs.msg.ObjectsInBoxes):               Detection results of current frame
            localized_object (object_analytics_msgs.msg.ObjectsInBoxes3D):   Localization results of current frame
            tracked_objects (object_analytics_msgs.msg.TrackedObjects):      Tracking results of current frame
        """
        objects = self._merge_results(detected_objects, localized_objects, tracked_objects)
        marker_array = self._build_marker_array(detected_objects.header, objects)
        self._pub.publish(marker_array)

    @staticmethod
    def _merge_results(detected_objects, localized_objects, tracked_objects):
        """Merge detection, localization and tracking results which have the same roi, roi should must be include in
          localization result

        Args:
            detected_objects (object_msgs.msg.ObjectsInBoxes):               Detection results of current frame
            localized_object (object_analytics_msgs.msg.ObjectsInBoxes3D):   Localization results of current frame
            tracked_objects (object_analytics_msgs.msg.TrackedObjects):      Tracking results of current frame
        Returns:
            ObjectItem list: list of ObjectItem in which each item is merged from detction, tracking and localization.
        """
        localized_map = {}
        for obj in localized_objects.objects_in_boxes:
            localized_map[Roi(obj.roi)] = obj

        detected_map = {}
        for obj in detected_objects.objects_vector:
            key = Roi(obj.roi)
            if key not in localized_map:
                continue
            detected_map[Roi(obj.roi)] = obj

        merged = []
        for obj in tracked_objects.tracked_objects:
            key = Roi(obj.roi)
            if key not in localized_map:
                continue
            merged.append(ObjectItem(tracked_objects.header, obj.roi, obj.id, detected_map[key].object,
                                     localized_map[key].min, localized_map[key].max))
        return merged

    @staticmethod
    def _build_marker_array(header, objects):
        """Build MarkerArray from given header and list of ObjectItem.

        Args:
            header (std_msgs.msg.Header):   Header of current processing frame
            objects (list):                 List of ObjectItem in which each item represents a merged result
        Returns:
            MarkerArray: Includes marker for cleaning preivous markers, marker for bouding box and marker for text.
        """
        marker_array = MarkerArray()

        clean_all_marker = Marker()
        clean_all_marker.header = header
        clean_all_marker.action = Marker.DELETEALL
        marker_array.markers.append(clean_all_marker)

        markers = [obj.linelist() for obj in objects]
        markers.extend([obj.min_text() for obj in objects])
        markers.extend([obj.max_text() for obj in objects])
        markers.extend([obj.name_id_text() for obj in objects])
        for idx, marker in enumerate(markers):
            marker.id = idx
        marker_array.markers.extend(markers)

        return marker_array


if __name__ == "__main__":
    rospy.init_node('visualization3d')
    SynchronizedSubscriber()
    rospy.spin()
