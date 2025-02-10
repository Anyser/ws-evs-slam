# EVS-SLAM

EVS-SLAM code for autonomous vehicle applications in dynamic environments, based on DeepLabv3 and ORBSLAM2, optimized for NVIDIA Jetson. GPL License for open use and contributions.

## Installation :wrench:  

This project requires specific hardware and software configurations for proper execution.

### Prerequisites :ballot_box_with_check:

- NVIDIA Jetson with JetPack 4.6.3, which includes support for CUDA 10.2.
- OpenCV 3.4.3 compiled with CUDA
- ZED SDK 3.7.7
- ROS Melodic
- PyTorch 1.10.1+cu102

### Installation Steps :gear:

#### JetPack 4.6.3:

1. Download and install JetPack 4.6.3 on the Jetson module. This version is necessary for CUDA 10.2 compatibility.
2. We suggest following https://docs.nvidia.com/sdk-manager/install-with-sdkm-jetson/index.html

#### OpenCV 3.4.3:

1. Uninstall any existing OpenCV version on your Jetson or computer.
2. Build and install OpenCV 3.4.

>[!NOTE]
>Verify that the installation works correctly with CUDA 10.2.
>You can check it on the Jetson using `jtop` following: https://github.com/rbonghi/jetson_stats


#### ZEDM Camera SDK:

1. Follow the instructions from https://www.stereolabs.com
2. Install the compatible SDK version 3.7.7.
3. Install pyzed along with the SDK installation.

#### ROS Melodic:

1. Install ROS Melodic.
2. Create a workspace for Python3 and CvBridge:
```console
mkdir ~/your_name_ws && cd ~/your_name_ws
catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6.m -DPYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.6m.so
catkin config --install

mkdir src
cd src
git clone -b melodic https://github.com/ros-perception/vision_opencv.git

cd ~/your_name_ws
catkin build_cv_bridge
source install/setup.bash --extend
```
>[!NOTE]
>This procedure may require additional steps since Melodic natively works with Python2.
>In some cases, you need to build and install ROS with Python3 support (suggested method): https://gist.github.com/MBaranPeker/3005ed281cf03922d641ba98bf8088eb#file-gistfile1-txt

#### Python3 Configuration :snake: :

In your Python3 environment, install the necessary libraries for PyTorch and CUDA 10.2:
   * python 3.6.9
   * torch 1.10,1+cu102 or 1.8.0a0+37c1f4a
   * numpy 1.19.5
   * torchvision 0.11.2+cu102 or 0.9.0a0+01dfa8e


#### Compiling seg_msg_ros:
1. Compile the seg_msg_ros package for the segmentation network and image publishing in ROS.

2. Download the model weights from https://drive.google.com/drive/folders/1Bf42Kbe-_AfrM0srMY7tZdCUDjqyx96-?usp=sharing

3. Modify their path in the _init_.py file.

```console
catkin build seg_msg_ros
```
4. Check that the net code and the camera are publishing:
```console
roscore
```
in another console
```console
/usr/bin/python3 /PATH/TO/YOUR/CATKIN_WS/src/seg_msg_ros/scripts/segmentacion/zedm_(stereo or rgbd)_pub_net.py
```

#### Installing EVS_SLAM :camera: :
1. Follow the steps described in [ORBSLAM2](https://github.com/raulmur/ORB_SLAM2).
   * Pangolin
  
>[!NOTE]
>Make sure the Pangolin version is compatible, for example v0.5 `git checkout tags/v0.5 -b`

2. Build EVS_SLAM
```console
./build.sh
```
3. Build evs_slam for ROS
```console
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/PATH/TO/CATKIN_WS/src/EVS_SLAM/Examples/ROS
```

4. Download the vocabulary and add it to Vocabulary 

after that:

```console
./build_ros_aarch64.sh
```
5. Check that the generated code subscribes and that VSLAM is working. The True or False variable indicates whether or not to map dynamic points:

```console
./ros_stereo_zed /PATH/TO/CATKIN_WS/src/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/lapyr/catkin_ws_slam/src/ORB_SLAM2_WOD/Examples/RGB-D/ZEDM_H.yaml True
```

## References
If this code is useful to you, please consider citing our work accordingly. Thank you for your support! :smile:




