# Getting started with ORB SLAM3 and SLAM using an android camera
This repo contains instructions on how to set up ORB SLAM 3 on Ubuntu 20.04 and using it with ROS noetic and other tools. It will include the instructions from the source repos and some errors I encountered that took some time to figure solve.

## ORB SLAM 3 Dependencies [guide](https://devpress.csdn.net/ubuntu/62f629af7e6682346618ab89.html)

<!--(### Dependencies [guide](https://devpress.csdn.net/ubuntu/62f629af7e6682346618ab89.html)-->
#### C++11 or C++0x Compiler
`sudo apt install build-essential`
or
`sudo apt install gcc g++ clang -y`

#### Pangolin
Download and installation instructions: [https://github.com/stevenlovegrove/Pangolin]

On Ubuntu 20.04, an error about Catch2 will be raised.  
Solution:  

`sudo add-apt-repository ppa:savoury1/build-tools`  
`sudo apt update`   
`sudo apt install catch2`  
This will allow for installation to complete successfully but the optional test won't work.

#### OpenCV
`sudo apt install libopencv-dev python3-opencv`  

Confirmation:  
In a terminal enter:  
`python3`   
`python3 -c "import cv2; print(cv2.__version__)"`  

#### Eigen
`sudo apt install libeigen3-dev`

#### DBoW2 and g2o (Included in Thirdparty folder)
In Ubuntu 20.04, these require the following dependency:   
Dependencies of g2o: `sudo apt install cmake libeigen3-dev libsuitesparse-dev qtdeclarative5-dev qt5-qmake libqglviewer-dev-qt5`   
Dependencies of DBoW2: `sudo apt install libboost-dev libboost-serialization-dev`    

#### Python 3.8
`sudo apt install python3.8`

#### Other dependencies
Glew: `sudo apt install libglew-dev`  
OpenGL: `sudo apt install libgl1-mesa-dev`
<!--Libboost: `sudo apt install libboost-dev libboost-serialization-dev`-->
Wayland  
&emsp;pkg-config: `sudo apt install pkg-config`  
&emsp;Wayland and EGL: `sudo apt install libegl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols`

Realsense issues:
Clone librealsense repo
`cd librealsense-2.54.2/`<br/>

`sudo apt install libssl-dev libusb-1.0.0-dev libudev-dev pkg-config libgtk-3-dev git wget cmake build-essential libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at`<br/>
or 
`sudo apt-get install libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev git wget cmake build-essential libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at`<br/>

`mkdir build && cd build`<br/>

`cmake ../ -DFORCE_RSUSB_BACKEND=true -DCMAKE_BUILD_TYPE=release -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true`<br/>

`sudo make uninstall && make clean && make -j8 && sudo make install`<br/>
  

## ORB SLAM 3
[Instructions](https://github.com/UZ-SLAMLab/ORB_SLAM3?tab=readme-ov-file#3-building-orb-slam3-library-and-examples)   
It is also possible to execute the commands in the `build.sh` file line-by-line or section-by-section.

Possible compiling issue fix:
`sed -i 's/++11/++14/g' CMakeLists.txt`

## Running EuRoc Datase
Download dataset
`./Examples/Monocular/mono_euroc ./Vocabulary/ORBvoc.txt ./Examples/Monocular/EuRoC.yaml ~/Downloads/MH_01/ ./Examples/Monocular/EuRoC_TimeStamps/MH01.txt  dataset-MH01_mono`<br/>

`./Examples/Stereo/stereo_euroc ./Vocabulary/ORBvoc.txt ./Examples/Stereo/EuRoC.yaml ~/Downloads/MH_01/ ./Examples/Stereo/EuRoC_TimeStamps/MH01.txt  dataset-MH01_stereo`

## ORB SLAM ROS
[Instructions](https://github.com/UZ-SLAMLab/ORB_SLAM3#7-ros-examples)
ROS directory may be located under Examples_old directory. Copy the directory to Examples directory.

Possible issues fix: 
`sed -i 's/++11/++14/g' CMakeLists.txt` for CMakeLists.txt in ROS directory.

`find_package(OpenCV <OPENCV version you installed> QUIET)`

<pre><p>include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/../../../
${PROJECT_SOURCE_DIR}/../../../include
${PROJECT_SOURCE_DIR}/../../../include/CameraModels
${PROJECT_SOURCE_DIR}/../../../Thirdparty/Sophus <!-- Add this line -->
${Pangolin_INCLUDE_DIRS}
)</p></pre>

Comment out or deleted the section "Node for monocular camera (Augmented Reality Demo)" if you don't intend to use it.

Run build_ros.sh. 

## ORB SLAM 3 ROS wrapper
Clone repo: [link](https://github.com/thien94/orb_slam3_ros)

Fix if the the ROS package can't be found after catkin build:
`source /home/<user>/catkin_ws/devel/setup.bash`<br/>
`roslaunch orbslam3_ros euroc_mono.launch`

## Droidcam
It offers footage streaming to a ROS node from an android device as an IP camera or using a USB cable. USB allows for higher quality frames and almost no lag.

Download android app from Playstore.
Linux setup [instructions](https://www.dev47apps.com/droidcam/linux/)

Possible issues on linux client:
1. `error adding adb forward`

`sudo adb kill-server`<br/>
`sudo adb start-server`

Launch app on phone
Connect phone to laptop using USB cable
Ensure debugging and tethering on the phone is on

NB: Note the video source indicated in the Linux client app. This will be used in the next section.

## Publishing phone video to a ROS node
Code and instructions under android_camera_to_ros_node directory


## Putting it all together

Terminal 1 - `roscore`<br/>
Terminal 2 - `rosrun ORB_SLAM3 Mono Vocabulary/ORBvoc.txt Examples/Monocular/EuRoC.yaml`<br/>
Terminal 3 - `roslaunch orb_slam3_ros euroc_mono.launch`<br/>
Terminal 4 - `python3 <path_to_androidCamera.py>/androidCamera.py`<br/>

## Customizing RVIZ menu
These options are in the orb_slam3_no_imu.rviz file of the ORBSLAM3 ROS wrapper package
