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

## Usage

The phones are running the app [hNode](https://play.google.com/store/apps/details?id=com.husarion.node&hl=en_GB), which establishes a network between them and the laptop [(article)](https://medium.com/husarion-blog/dont-buy-expensive-sensors-for-your-robot-use-your-smartphone-24380eab521). After being registered on the same [network](https://app.husarnet.com/network/849), running ```rostopic list``` shows the topics the phones are publishing with their sensor data.

To calibrate the phone cameras, we used the Matlab function [cameraCalibrator](https://uk.mathworks.com/help/vision/ref/cameramatrix.html) to obtain the camera projection matrix P using a 8x6 checkerboard with 24mm square size and 40 samples. This gives us the FocalLength (f in the matrix) and the Principal points (ox and oy in the matrix).

On the laptop, several commands should be run.

Make sure the ```~/.bashrc``` or  ```~/.zshrc``` has the following lines to ensure ROS compiles and that the Husarion networks connects correctly:
```
source /opt/ros/melodic/setup.bash
source <ROS workspace dir>/devel/setup.sh

export ROS_IPV6=on
export ROS_MASTER_URI=http://master:11311
export ROS_HOSTNAME=master
```

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
rosrun video_transport video_subscriber_sync 
```
to combine the phone camera frames with the lidar cloudpoints.

For now, the 4 phones can be calibrated against the lidar separately. To do so, edit [calibration-node.cpp](but_calibration_camera_velodyne/src/calibration-node.cpp) and [calibration.yaml](but_calibration_camera_velodyne/conf/calibration.yaml) to select the phone in question. Then, run:
```
roslaunch but_calibration_camera_velodyne calibration_fine.launch pixel:="[PIXEL_NO]"
```
The results from this calibration should be saved in [coloring.yaml](but_calibration_camera_velodyne/conf/coloring.yaml).

To generate all 4 colored cloudpoints from the 4 phones, run:
```
roslaunch but_calibration_camera_velodyne coloring.launch
```
And then, to visualise them, use rviz and listen to all 4 topics (/velodyne_colored_points_1-4):
```
rosrun rviz rviz -f velodyne
```

## Troubleshooting

If things are suddenly not working and one wants to start the build afresh, run:

```
cd <ROS workspace dir>
rm -rf build/
rm -rf devel/
catkin_make install
catkin_make
```

If the phones and lidar don't sync properly, check their exact system time. If it's not the same, down to the second, one may have sync issues. For reference, use the [official US atomic time website](time.gov) to match their system times.

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
  - [hNode](https://play.google.com/store/apps/details?id=com.husarion.node&hl=en_GB): ROS node running in each of the phones -> publish phone sensor data (incl cameras)
  - [velodyne](https://github.com/ros-drivers/velodyne): driver for the VLP16, publishes the 3D cloudpoints over ethernet to ```/velodyne_points```.
  - phone_streams: listens to compressed image data from phones, uncompress it, and publish raw in new topic
  - video_transport: listens to both the raw cameras topics and the cloudpoints topics, waits until it has data from all 5 within the same time frame, and then superimposes the cloudpoints on top of the streams.
  - [but_calibration_camera_velodyne](https://github.com/robofit/but_velodyne/tree/master/but_calibration_camera_velodyne): node to callibrate the coordinate systems of the Lidar and 4 cameras.

### Server (Recognition System)
  - TBD

### Visualisation
  - Is currently in video_transport, using [cv2](https://opencv.org/).
  - Check out NGINX w/ RTMP?
