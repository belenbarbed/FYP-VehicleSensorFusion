# Vehicle Visual and Distance Sensor Fusion and Real-time Processing

Supervisor: Soteris Demetriou

## Goal
To emulate part of the functionality of autonomous vehicles. You will build a car mount which can collect visual and distance information, and detect objects around the vehicle as well as their angle and distance from it. It should also be able to display in real time the captured information.

## Literature
[Intro paper](http://seclab.illinois.edu/wp-content/uploads/2018/06/demetriou2018codrive.pdf)

[Callibration paper](https://pdfs.semanticscholar.org/ed15/5d1a146e0cba6be98fd7128461439f88732a.pdf)

  - [Implementation](https://github.com/laboshinl/but_calibration_camera_velodyne)

## Links
[GitHub Repo](https://github.com/belenbarbed/FYP-VehicleSensorFusion)

[Google Drive Folder](https://drive.google.com/drive/folders/1wz1pCsgVdYGVe1YnCs0_oeLTOG-cAbBa?usp=sharing)

## Usage

The phones are running the app [hNode](https://medium.com/husarion-blog/dont-buy-expensive-sensors-for-your-robot-use-your-smartphone-24380eab521), which establishes a network between them and the laptop. After being registered on the same [network](https://app.husarnet.com/network/849), running ```rostopic list``` shows the topics the phones are publishing with their sensor data.

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

We are currently working on the callibration of these two systems.

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
  - hNode: ROS node running in each of the phones -> publish phone sensor data (incl cameras)
  - [velodyne](https://github.com/ros-drivers/velodyne): driver for the VLP16, publishes the 3D cloudpoints to ```/velodyne_points```.
  - phone_streams: listens to compressed image data from phones, uncompress it, and publish raw in new topic
  - video_transport: listens to both the raw cameras topics and the cloudpoints topics, waits until it has data from all 5 within the same time frame, and the superimposes the cloudpoints on top of the streams.
  - [but_calibration_camera_velodyne](https://github.com/robofit/but_velodyne/tree/master/but_calibration_camera_velodyne): node to callibrate the coordinate systems of the Lidar and 4 cameras.

### Server (Recognition System)
  - TBD

### Visualisation
  - NGINX w/ RTMP?
