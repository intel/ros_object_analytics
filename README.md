# ros_object_analytics
Object Analytics (OA) is ROS wrapper for realtime object detection, localization and tracking.
These packages aim to provide real-time object analyses over RGB-D camera inputs, enabling ROS developer to easily create amazing robotics advanced features, like intelligent collision avoidance and semantic SLAM. It consumes [sensor_msgs::PointClould2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html) data delivered by RGB-D camera, publishing topics on [object detection](https://github.com/intel/object_msgs), [object tracking](https://github.com/intel/ros_object_analytics/tree/master/object_analytics_msgs), and [object localization](https://github.com/intel/ros_object_analytics/object_analytics_msgs) in 3D camera coordination system.

OA keeps integrating with various "state-of-the-art" algorithms.
* Object detection offload to GPU, [ros_opencl_caffe](https://github.com/intel/ros_opencl_caffe), with Yolo v2 model and [OpenCL Caffe](https://github.com/01org/caffe/wiki/clCaffe#yolo2-model-support) framework
* Object detection offload to VPU, [ros_intel_movidius_ncs (devel branch)](https://github.com/intel/ros_intel_movidius_ncs/tree/devel/), with MobileNet SSD model and Caffe framework

## System Requirements
We support Ubuntu Linux Xenial Xerus 16.04 on 64-bit. We not support Mac OS X and Windows.


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
  ```
  sudo apt-get install ros-kinetic-desktop-full
  ```

  Other ROS packages

  * [object_msgs](https://github.com/intel/object_msgs)
  ```
  mkdir ~/ros_ws/src -p
  cd ~/ros_ws/src
  git clone https://github.com/intel/object_msgs
  ```

  * Build OpenCV3 & opencv-contrib 3.3 (OA depends on tracking feature from OpenCV Contrib 3.3. OpenCV 3.3 is not integrated in ROS2 Crystal release, need to build and install Opencv3 with contrib from source to apply tracking feature)
  ```
  # Build and Install OpenCV3 with opencv-contrib
  mkdir ${HOME}/opencv
  cd ${HOME}/opencv
  git clone https://github.com/opencv/opencv.git -b 3.3.0
  git clone https://github.com/opencv/opencv_contrib.git -b 3.3.0
  mkdir opencv/build -p
  cd opencv/build
  cmake -DOPENCV_EXTRA_MODULES_PATH=${HOME}/opencv/opencv_contrib/modules -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_opencv_cnn_3dobj=OFF ..
  make -j8
  sudo make install
  sudo ldconfig
  ```

  * [ros_intel_movidius_ncs](https://github.com/intel/ros_intel_movidius_ncs)(To install ros_intel_movidius_ncs or opencl_caffe as demand)

  ```
  # build ncsdk
  mkdir ~/code
  cd ~/code
  git clone https://github.com/movidius/ncsdk.git
  git clone https://github.com/movidius/ncappzoo.git
  cd ncsdk/
  make install
  ln -sf ~/code/ncappzoo /opt/movidius/ncappzoo
  cd /opt/movidius/ncappzoo/caffe/SSD_MobileNet && make


  # build ros_intel_movidius_ncs
  cd ~/ros_ws/src
  git clone https://github.com/intel/ros_intel_movidius_ncs.git
  cp -rf ros_intel_movidius_ncs/data/labels/* /opt/movidius/ncappzoo/data/ilsvrc12/
  cd ~/ros_ws
  source /opt/ros/kinetic/setup.bash
  catkin_make_isolated

  ```
  * [opencl_caffe](https://github.com/intel/ros_opencl_caffe) (To install ros_intel_movidius_ncs or opencl_caffe as demand)
  ```
  # install OpenCL
  mkdir ~/code
  cd ~/code
  wget http://registrationcenter-download.intel.com/akdlm/irc_nas/11396/SRB4.1_linux64.zip
  unzip SRB4.1_linux64.zip
  sudo usermod -a -G video `users`
  sudo apt-get install openssl libnuma1 libpciaccess0
  sudo apt-get install xz-utils
  mkdir intel-opencl
  tar -C intel-opencl -xxf intel-opencl-r4.1-61547.x86_64.tar.xz
  tar -C intel-opencl -xvf intel-opencl-devel-r4.1-61547.x86_64.tar.xz
  tar -C intel-opencl -xvf intel-opencl-cpu-r4.1-61547.x86_64.tar.xz
  sudo cp -rf intel-opencl/* /
  sudo ldconfig

  # install MKL
  mkdir -p ~/code/MKL
  cd ~/code/MKL
  wget -c http://registrationcenter-download.intel.com/akdlm/irc_nas/tec/12070/l_mkl_2018.0.128.tgz
  tar xvf l_mkl_2018.0.128.tgz
  cd l_mkl_2018.0.128
  sed -i 's/ACCEPT_EULA=decline/ACCEPT_EULA=accept/g' silent.cfg
  sudo ./install.sh -s ./silent.cfg || true
  echo export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/opt/intel/mkl/lib/intel64_lin >> ~/.bashrc
  source ~/.bashrc

  # install viennacl-dev
  cd ~/code
  sudo apt-get install libprotobuf-dev libleveldb-dev libsnappy-dev libopencv-dev libhdf5-serial-dev protobuf-compiler
  sudo apt-get install --no-install-recommends libboost-all-dev
  sudo apt-get install libopenblas-dev liblapack-dev libatlas-base-dev
  sudo apt-get install libgflags-dev libgoogle-glog-dev liblmdb-dev
  git clone https://github.com/viennacl/viennacl-dev.git
  cd viennacl-dev
  mkdir build && cd build
  cmake -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DCMAKE_INSTALL_PREFIX=$HOME/local -DOPENCL_LIBRARY=/opt/intel/opencl/libOpenCL.so ..
  make -j4
  make install

  # install isacc
  cd ~/code
  git clone https://github.com/intel/isaac
  cd isaac
  mkdir build && cd build
  cmake -DCMAKE_INSTALL_PREFIX=$HOME/local .. && make -j4
  make install

  # install clCaffe
  cd ~/code
  git clone https://github.com/01org/caffe clCaffe
  cd clCaffe
  git checkout inference-optimize
  export PYTHONPATH=$PYTHONPATH:$HOME/code/clCaffe/python
  mkdir build && cd build
  export ISAAC_HOME=$HOME/local
  cmake .. -DUSE_GREENTEA=ON -DUSE_CUDA=OFF -DUSE_INTEL_SPATIAL=ON -DBUILD_docs=0 -DUSE_ISAAC=ON -DviennaCL_INCLUDE_DIR=$HOME/local/include -DBLAS=mkl -DOPENCL_LIBRARIES=/opt/intel/opencl/libOpenCL.so -DOPENCL_INCLUDE_DIRS=/opt/intel/opencl/include
  make -j4
  export CAFFE_ROOT=$HOME/code/clCaffe
  sudo pip install --upgrade pip (pip >= 9.0.1 is required by matplotlib)
  sudo pip install scikit-image
  sudo pip install protobuf
  export PYTHONPATH=${CAFFE_ROOT}/python/
  cd ..
  wget https://pjreddie.com/media/files/yolo-voc.weights -O models/yolo/yolo416/yolo.weights
  python models/yolo/convert_yolo_to_caffemodel.py
  python tools/inference-optimize/model_fuse.py --indefinition models/yolo/yolo416/yolo_deploy.prototxt --outdefinition models/yolo/yolo416/fused_yolo_deploy.prototxt --inmodel models/yolo/yolo416/yolo.caffemodel --outmodel models/yolo/yolo416/fused_yolo.caffemodel
  sudo ln -s $HOME/code/clCaffe /opt/clCaffe
  echo export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/opt/clCaffe/build/lib >> ~/.bashrc
  source ~/.bashrc

  # install ros_opencl_caffe
  cd ~/ros_ws/src
  git clone https://github.com/intel/ros_opencl_caffe
  cd ~/ros_ws
  source /opt/ros/kinetic/setup.sh
  catkin_make_isolated
  cp ~/ros_ws/src/ros_opencl_caffe/opencl_caffe/resources/voc.txt /opt/clCaffe/data/yolo

  ```

## extra camera dependencies
  RGB-D camera with Intel RealSense D400 Series
  * [librealsense2 tag v2.9.1](https://github.com/IntelRealSense/librealsense/tree/v2.9.1)
  ```
  # Build librealsense (ref to https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)
  sudo apt-get install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev libglfw3-dev
  cd ~/code
  git clone https://github.com/IntelRealSense/librealsense -b v2.9.1
  mkdir -p ~/code/librealsense/build
  cd ~/code/librealsense/build
  cmake ../
  make -j8
  sudo make install
  cd ../
  sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
  sudo udevadm control --reload-rules && udevadm trigger
  ```

  * [realsense_ros_camera tag 2.0.2](https://github.com/intel-ros/realsense/tree/2.0.2)
  ```
  # Build realsense_ros_camera
  sudo apt-get install ros-kinetic-rgbd-launch
  mkdir ~/ros_ws/src -p
  cd ~/ros_ws/src
  git clone https://github.com/intel-ros/realsense -b 2.0.2
  cd ~/ros_ws
  source /opt/ros/kinetic/setup.bash
  catkin_make_isolated --source src/realsense
  ```

  * launch RealSense Node
  ```
  source /opt/ros/kinetic/setup.bash
  source ~/ros_ws/devel_isolated/setup.bash
  roslaunch realsense_ros_camera rs_rgbd.launch
  ```

## build object_analytics
  ```
  cd ~/ros_ws/src
  git clone https://github.com/intel/ros_object_analytics
  cd ~/ros_ws
  source /opt/ros/kinetic/setup.bash
  source ~/ros_ws/devel_isolated/setup.bash
  catkin_make_isolated
  ```

## command to launch object_analytics
* source ros environment
  ```
  source /opt/ros/kinetic/setup.bash
  source ~/ros_ws/devel_isolated/setup.bash
  ```
* launch with OpenCL caffe as detection backend
  ```bash
  roslaunch object_analytics_launch analytics_opencl_caffe.launch
  ```
* launch with Movidius NCS as detection backend
  ```bash
  roslaunch object_analytics_launch analytics_movidius_ncs.launch
  ```

  Frequently used options
  * **input_points** Specify arg "input_points" for the name of the topic publishing the [sensor_msgs::PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html) messages by RGB-D camera. Default is "/camera/depth_registered/points" (topic compliant with [ROS OpenNI launch](http://wiki.ros.org/openni_launch))
  * **aging_th** Specifiy tracking aging threshold, number of frames since last detection to deactivate the tracking. Default is 16.
  * **probability_th** Specify the probability threshold for tracking object. Default is "0.5".
  ```bash
  roslaunch object_analytics_launch analytics_movidius_ncs.launch aging_th:=30 probability_th:="0.3"
  ```

## published topics
  object_analytics/rgb ([sensor_msgs::Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))

  object_analytics/pointcloud ([sensor_msgs::PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html))

  object_analytics/localization ([object_analytics_msgs::ObjectsInBoxes3D](https://github.com/intel/ros_object_analytics/tree/master/object_analytics_msgs/msg))

  object_analytics/tracking ([object_analytics_msgs::TrackedObjects](https://github.com/intel/ros_object_analytics/tree/master/object_analytics_msgs/msg))

  object_analytics/detection ([object_msgs::ObjectsInBoxes](https://github.com/intel/object_msgs/tree/master/msg))


## visualize tracking and localization results on RViz
  Steps to enable visualization on RViz are as following
  ```bash
  source /opt/ros/kinetic/setup.bash
  source ~/ros_ws/devel_isolated/setup.bash
  roslaunch object_analytics_visualization rviz.launch
  ```

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

###### *ROS 2 Object Analytics: https://github.com/intel/ros2_object_analytics*
###### *Any security issue should be reported using process at https://01.org/security*
