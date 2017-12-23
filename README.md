# ros_object_analytics
ROS wrapper for realtime object detection, localization and tracking

## compiling dependencies
  ROS packages from [ros-kinetic-desktop-full](http://wiki.ros.org/kinetic/Installation/Ubuntu)
  * roscpp
  * nodelet
  * std_msgs
  * sensor_msgs
  * geometry_msgs
  * dynamic_reconfigure
  * pcl_conversions
  * cv_bridge
  * libpcl-all
  * libpcl-all-dev
  * ros-kinetic-opencv3

  Other ROS packages
  * [object_msgs](https://github.com/intel/object_msgs)
  * [opencl_caffe](https://github.com/intel/ros_opencl_caffe)
  * [object_detect_launch](https://github.com/intel/ros_object_detect_launch)

  NOTE: In older version of "ros-kinetic-opencv3" where OpenCV 3.2.0 was used, self-built opencv_tracking is needed. While this's no more necessary since OpenCV 3.3 integrated. Check the OpenCV version from "/opt/ros/kinetic/share/opencv3/package.xml"
  * [opencv_tracking](https://github.com/opencv/opencv_contrib) tag 3.2.0

## build and test
  * to build
  ```bash
  cd ${ros_ws} # "ros_ws" is the catkin workspace root directory where this project is placed in
  catkin_make
  ```
  * to test
  ```bash
  catkin_make run_tests
  ```

  * to install
  ```bash
  catkin_make install
  ```

## extra running dependencies
  RGB-D camera
  * [librealsense2 tag v2.8.1](https://github.com/IntelRealSense/librealsense/tree/v2.8.1) and [realsense_ros_camera tag 2.0.1](https://github.com/intel-ros/realsense/tree/2.0.1) if run with Intel RealSense D400
  ```
  roslaunch realsense_ros_camera rs_camera.launch enable_pointcloud:=true enable_sync:=true enable_infra1:=false enable_infra2:=false
  ```
  * [openni_launch](http://wiki.ros.org/openni_launch) or [freenect_launch](http://wiki.ros.org/freenect_launch) and their dependencies if run with Microsoft XBOX 360 Kinect
  ```bash
  roslaunch openni_launch openni.launch
  ```
  * [ros_astra_camera](https://github.com/orbbec/ros_astra_camera) if run with Astra Camera
  ```bash
  roslaunch astra_launch astra.launch
  ```

## command sample to launch object_analytics
  Specify arg "input_points" for the name of the topic publishing the [sensor_msgs::PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html) messages by RGB-D camera, default "/camera/depth_registered/points"
  ```bash
  roslaunch object_analytics_launch analytics.launch input_points:=/camera/points
  ```

  There are two choices for 2d detection, one is [clCaffe](https://github.com/intel/ros_opencl_caffe) another is [Movidius NCS](https://github.com/intel/ros_intel_movidius_ncs), Movidius NCS is the default, and can switch to clCaffe by appending 'detect_impl:=opencl_caffe' ros argument, like below:
  ```bash
  roslaunch object_analytics_launch analytics.launch detect_impl:=opencl_caffe
  ```

## published topics
  ```bash
  object_analytics/localization (object_analytics_msgs::ObjectsInBoxes3D)
  object_analytics/tracking (object_analytics_msgs::TrackedObjects)
  ```
## rostest
  The roslaunch files with ".test" surfix will launch the test node and all dependents, including camera, detection nodelet, and object_analytics.
  * run tracking test without visual outputs
  ```bash
  catkin_make
  rostest object_analytics_nodelet mtest_tracking.test
  ```
  * run tracking test with visual outputs
  ```bash
  catkin_make clean --pkg object_analytics_nodelet
  catkin_make -DMTEST_TRACKING_ENABLE_VIEW=ON --pkg object_analytics_nodelet
  # to launch the test with realsense camera, specify arg "input_points"
  rostest object_analytics_nodelet mtest_tracking.test input_points:=/camera/points
  # to launch the test with astra camera, specify arg "camera"
  rostest object_analytics_nodelet mtest_tracking.test camera:=2
  ```

## visualize tracking and localization results on RViz
  Steps to enable visualization on RViz are as following
  ```bash
  roslaunch object_analytics_visualization rviz.launch
  ```

###### *Any security issue should be reported using process at https://01.org/security*
