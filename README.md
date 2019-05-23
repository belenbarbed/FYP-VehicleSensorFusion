# Vehicle Visual and Distance Sensor Fusion and Real-time Processing

Supervisor: Soteris Demetriou

## Authors

[Belen Barbed](https://github.com/belenbarbed)

[Kaifeng Yan](https://github.com/Kai-2333)

[Flora Qin](https://github.com/FloraQin0325)

## Goal
To emulate part of the functionality of autonomous vehicles. You will build a car mount which can collect visual and distance information, and detect objects around the vehicle as well as their angle and distance from it. It should also be able to display in real time the captured information.

## Literature
[Intro paper](http://seclab.illinois.edu/wp-content/uploads/2018/06/demetriou2018codrive.pdf)

[Callibration paper](https://pdfs.semanticscholar.org/ed15/5d1a146e0cba6be98fd7128461439f88732a.pdf)

  - [Implementation](https://github.com/robofit/but_velodyne/tree/master/but_calibration_camera_velodyne)

## Links
[GitHub Repo](https://github.com/belenbarbed/FYP-VehicleSensorFusion)

[Google Drive Folder](https://drive.google.com/drive/folders/1wz1pCsgVdYGVe1YnCs0_oeLTOG-cAbBa?usp=sharing)

## First time setup

### Phones

The phones are running the app [hNode](https://play.google.com/store/apps/details?id=com.husarion.node&hl=en_GB), which establishes a network between them and the PC [(article)](https://medium.com/husarion-blog/dont-buy-expensive-sensors-for-your-robot-use-your-smartphone-24380eab521). After being registered on the same [network](https://app.husarnet.com/network/849), together with the PC, running ```rostopic list``` shows the topics the phones are publishing with their sensor data.

### PC (master node)

Add the PC to the Husarnet network using:
```
curl https://install.husarnet.com/install.sh | sudo bash
sudo husarnet websetup
```
Follow the link returned to configure your device.

Make sure the ```~/.bashrc``` or  ```~/.zshrc``` has the following lines to ensure ROS compiles and that the Husarion networks connects correctly:
```
source /opt/ros/melodic/setup.bash
source <ROS workspace dir>/devel/setup.sh

export ROS_IPV6=on
export ROS_MASTER_URI=http://master:11311
export ROS_HOSTNAME=master
```

After pulling this repo into the src of a catkin workspace, remember to also pull the git submodules present: ```velodyne``` and ```vision_opencv``` with:
```
git submodule update --init --recursive
```
It is recommended to remove the ```build/``` and ```devel/``` folders and build the workspace from scratch at the beginning, using ```catkin build```.

### Velodyne

Follow the [ROS integration instructions](http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16)

## Calibration

To calibrate the phone cameras, we used the Matlab function [cameraCalibrator](https://uk.mathworks.com/help/vision/ref/cameramatrix.html) to obtain the camera projection matrix P using a 8x6 checkerboard with 24mm square size and 40 samples. This gives us the FocalLength (f in the matrix) and the Principal points (ox and oy in the matrix).

TODO: Add pic of matrix

The 4 phones can be calibrated against the lidar separately. To do so, run:
```
roslaunch but_calibration_camera_velodyne calibration_fine.launch pixel:="[PIXEL_NO]"
```
with PIXEL_NO being the phone one wants to calibrate (1-4). The results from this calibration should be saved in [coloring.yaml](but_calibration_camera_velodyne/conf/coloring.yaml).

## Usage

On the PC, several commands should be run.

On the first tab, run ```roscore```.

Then, run the package that listens to the phone topics and formats the image into a cv2-acceptable format with:
```
rosrun phone_streams phone_streams_publisher.py
```

To make the Velodyne lidar start posting cloudpoint data, run:
```
roslaunch velodyne_pointcloud VLP16_points.launch
```

After that, one can run
```
rosrun face_detection face_detection_py.py
```
to detect faces (instead of cars, for now) in all 4 camera streams and display them all.

TODO: visualisation w/ distance info.

## Troubleshooting

If things are suddenly not working and one wants to start the build afresh, run:
```
cd <ROS workspace dir>
catkin clean
catkin build
```
Then remember to source in ALL open tabs:
```
source devel/setup.sh
```

If the phones and lidar don't sync properly, check their exact system time. If it's not the same, down to the second, one may have sync issues. For reference, use the [official US atomic time website](time.gov) to match their system times.

Other useful troubleshooting guides:
  - [Using cvBridge with python3](https://stackoverflow.com/questions/49221565/unable-to-use-cv-bridge-with-ros-kinetic-and-python3)
  - [cv2+numpy import problems](https://stackoverflow.com/questions/20518632/importerror-numpy-core-multiarray-failed-to-import)
  - [no module named em](https://answers.ros.org/question/257757/importerror-no-module-named-em-error/)

## Project Sections

### Equipment
  - Lidar: Velodyne VLP-16 + mount
    - [Product page](https://velodynelidar.com/vlp-16-lite.html)
    - [User manual](https://velodynelidar.com/docs/manuals/63-9243%20REV%20D%20MANUAL,USERS,VLP-16.pdf)
    - [ROS integration instructions](http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16)
  - 4 Google Pixel 2 XL + phone mounts
  - Mount to vehicle
    - Circular & symmetric base
    - Aluminium extrusions to hold to car
    - Metal stand-offs to support base (4) and lidar (1)
    - 4 holes for phone mounts

### ROS
  - [but_calibration_camera_velodyne](https://github.com/robofit/but_velodyne/tree/master/but_calibration_camera_velodyne): node to callibrate the coordinate systems of the Lidar and 4 cameras.
  - face_detection: uses [ageitgey's OpenCV implementation](https://github.com/ageitgey/face_recognition) to detect all faces per frame and overlays a red rectangle on top of the found faces. Displays all 4 video feeds too.
  - phone_streams: listens to compressed image data from phones, uncompresses it, and publishes the raw image in new topic.
  - [hNode](https://play.google.com/store/apps/details?id=com.husarion.node&hl=en_GB): ROS node running in each of the phones to publish all phone sensor data (incl cameras) as ROS topics.
  - [velodyne](https://github.com/ros-drivers/velodyne): driver for the VLP16, publishes the 3D cloudpoints over ethernet to ```/velodyne_points```.
  - [vision_opencv](https://github.com/ros-perception/vision_opencv.git): contains cv_bridge to use with python3 nodes.
  - visualization: TODO
  

### Recognition System (server)
  - Face detection: in face_detection package using [ageitgey's OpenCV implementation](https://github.com/ageitgey/face_recognition)
  - Car detection: TODO (maybe YOLO?)

### Visualisation
  - Is currently in face_detection, using [cv2](https://opencv.org/).
  - TODO: Check out NGINX w/ RTMP?
