# Collecting Data with OpenPose and Kinect 2.0
This package contains a simple data collector 
allows you to collect both keypoint data and 
 rendered images from 
[OpenPose](https://github.com/CMU-Perceptual-Computing-Lab/openpose) 
using the Kinect 2.0 camera.
### Dependencies
1. [ROS Kinetic](http://wiki.ros.org/kinetic)
2. [iai_kinect2](https://github.com/code-iai/iai_kinect2)
3. [OpenPose](https://github.com/CMU-Perceptual-Computing-Lab/openpose)
4. [ros-virtual-cam](https://github.com/lucasw/ros-virtual-cam), follow these instructioins 
to install
```shell
cd ~/catkin_ws/src  # or wherever catkin_ws is
git clone https://github.com/lucasw/ros-virtual-cam
cd ..
catkin build virtual_cam
```
### Before you run the code...
* Make sure you install the attached **openpose_msg** package 
in your catkin workspace so that you can use it in rospy
```shell
cp openpose_msg catkin_ws_location/catkin_ws/src
cd catkin_ws_location/catkin_ws
catkin build
```
* **Before you run your code for the first time**
```shell
sudo modprobe v4l2loopback video_nr=1
```
This is to create a fake camera so that Kinect 2 can stream to it.

To check that it worked, run the following in the terminal:
```shell
v4l2-ctl -D -d /dev/video1
```
It should look like this:
```shell
Driver Info (not using libv4l2):
    Driver name   : v4l2 loopback
    Card type     : Dummy video device (0x0000)
    Bus info      : v4l2loopback:0
    Driver version: 0.8.0
    Capabilities  : 0x05000003
        Video Capture
        Video Output
        Read/Write
        Streaming
```
Sometimes when your computer reboots, the generated
`video1` file gets deleted, so you would need to run
`sudo modprobe v4l2loopback video_nr=1` again.
### Running the code
1. Go into `parameters.py` and change the values of the
parameters to your desired values.
2. Run ```python listener.py``` using **Python 2**.
3. During the data collection process, press `b` to begin
collecting a series of action data, press `e` to finish
 action collection. Press `q` to shut down all the processes
 at any time.