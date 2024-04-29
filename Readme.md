# Master Thesis ~ Nerf

## Installation & Setup

A Dockerfile and installation instructions are provided in the [Docker](Docker/) directory. 

Follow the steps in that [Readme](Docker/Readme.md) to set up your Docker environment which encapsulates all required dependencies.

---

## Input Data

In case of live data streaming from the drone or robot is not available, you can use either of the following approaches:

1. **Rosbags**: Utilize existing ROS bags that contain the necessary image or video data.
2. **Image Publisher**: Deploy the provided image publisher ROS package to stream or publish images from a folder.

---

## Image Publisher

To use the image_publisher ROS package, follow these steps:

Installation:
1. Move the ```image_publisher``` folder to your Catkin workspace's src folder ```'~/catkin_ws/src'```.
2. Navigate to your Catkin workspace and compile the package:
```bash
cd ~/catkin_ws/
catkin_make
```
3. To publish images at a rate of 30 fps to the ```'/usb_cam/image_raw/'``` topic, use the following command:
```bash
rosrun image_publisher image_publisher.py -i /root/code/data/juice/ -t /usb_cam/image_raw/ -r 30.0
```

For detailed help with command-line arguments, use the following command:
```bash
rosrun image_publisher image_publisher.py -h
```

> [!NOTE]  
> * Ensure that the camera is calibrated before using the image publisher or a rosbag.
> * The camera's intrinsic parameters are crucial for the subsequent steps.
> * In the case of Mono-Inertial data(video with IMU), [Kalibr visual-intertial calibration](https://github.com/ethz-asl/kalibr) could be used for calibration.
