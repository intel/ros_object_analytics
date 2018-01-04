# ros_object_analytics
Object Analytics (OA) is ROS wrapper for realtime object detection, localization and tracking.
These packages aim to provide real-time object analyses over RGB-D camera inputs, enabling ROS developer to easily create amazing robotics advanced features, like intelligent collision avoidance and semantic SLAM. It consumes [sensor_msgs::PointClould2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html) data delivered by RGB-D camera, publishing topics on [object detection](https://github.com/intel/object_msgs), [object tracking](https://github.com/intel/ros_object_analytics/tree/master/object_analytics_msgs), and [object localization](https://github.com/intel/ros_object_analytics/object_analytics_msgs) in 3D camera coordination system.

OA keeps integrating with various "state-of-the-art" algorithms.
* Object detection offload to GPU, [ros_opencl_caffe](https://github.com/intel/ros_opencl_caffe), with Yolo v2 model and [OpenCL Caffe](https://github.com/01org/caffe/wiki/clCaffe#yolo2-model-support) framework
* Object detection offload to VPU, [ros_intel_movidius_ncs (devel branch)](https://github.com/intel/ros_intel_movidius_ncs/tree/devel/), with MobileNet SSD model and Caffe framework

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
  * [ros_intel_movidius_ncs](https://github.com/intel/ros_intel_movidius_ncs) or [opencl_caffe](https://github.com/intel/ros_opencl_caffe)

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
  roslaunch realsense_ros_camera rs_camera.launch enable_pointcloud:=true enable_sync:=true
  ```
  * [openni_launch](http://wiki.ros.org/openni_launch) or [freenect_launch](http://wiki.ros.org/freenect_launch) and their dependencies if run with Microsoft XBOX 360 Kinect
  ```bash
  roslaunch openni_launch openni.launch
  ```
  * [ros_astra_camera](https://github.com/orbbec/ros_astra_camera) if run with Astra Camera
  ```bash
  roslaunch astra_launch astra.launch
  ```

## command to launch object_analytics
* launch with OpenCL caffe as detection backend
   ```bash
   roslaunch object_analytics_launch analytics_opencl_caffe.launch
   ```
* launch with Movidius NCS as detection backend
   ```bash
   roslaunch object_analytics_launch analytics_opencl_caffe.launch
   ```

  Frequently used options
  * **input_points** Specify arg "input_points" for the name of the topic publishing the [sensor_msgs::PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html) messages by RGB-D camera. Default is "/camera/depth_registered/points". For realsense it is "/camera/points".
  ```bash
  roslaunch object_analytics_launch analytics_movidius_ncs.launch input_points:=/camera/points
  ```

## published topics
  object_analytics/rgb ([sensor_msgs::Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))

  object_analytics/pointcloud ([sensor_msgs::PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html))

  object_analytics/localization ([object_analytics_msgs::ObjectsInBoxes3D](https://github.com/intel/ros_object_analytics/tree/master/object_analytics_msgs/msg))

  object_analytics/tracking ([object_analytics_msgs::TrackedObjects](https://github.com/intel/ros_object_analytics/tree/master/object_analytics_msgs/msg))

  object_analytics/detection ([object_msgs::ObjectsInBoxes](https://github.com/intel/object_msgs/tree/master/msg))


## KPI of differnt detection backends
<table>
    <tr>
        <td></td>
        <td>topic</td>
        <td>fps</td>
        <td>latency <sup>sec</sup></td>
    </tr>
    <tr>
        <td rowspan='4'>OpenCL Caffe</td>
    </tr>
    <tr>
        <td>localization</td>
        <td>6.63</td>
        <td>0.23</td>
    </tr>
    <tr>
        <td>detection</td>
        <td>8.88</td>
        <td>0.17</td>
    </tr>
    <tr>
        <td>tracking</td>
        <td>12.15</td>
        <td>0.33</td>
    </tr>
    <tr>
        <td rowspan='4'>Movidius NCS</sup></td>
    </tr>
    <tr>
        <td>localization</td>
        <td>7.44</td>
        <td>0.21</td>
    </tr>
    <tr>
        <td>detection</td>
        <td>10.5</td>
        <td>0.15</td>
    </tr>
    <tr>
        <td>tracking</td>
        <td>13.85</td>
        <td>0.24</td>
    </tr>
</table>

* CNN model of Movidius NCS is MobileNet
* Hardware: Intel(R) Xeon(R) CPU E3-1275 v5 @3.60GHz, 32GB RAM, Intel(R) RealSense R45

## visualize tracking and localization results on RViz
  Steps to enable visualization on RViz are as following
  ```bash
  roslaunch object_analytics_visualization rviz.launch
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

###### *Any security issue should be reported using process at https://01.org/security*
