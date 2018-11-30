# Vehicle Visual and Distance Sensor Fusion and Real-time Processing

Supervisor: Soteris Demetriou

## Goal
To emulate part of the functionality of autonomous vehicles. You will build a car mount which can collect visual and distance information, and detect objects around the vehicle as well as their angle and distance from it. It should also be able to display in real time the captured information.

## Literature
[Intro paper](http://seclab.illinois.edu/wp-content/uploads/2018/06/demetriou2018codrive.pdf)

[Callibration paper](https://pdfs.semanticscholar.org/ed15/5d1a146e0cba6be98fd7128461439f88732a.pdf)

[Possible implementation](https://github.com/laboshinl/but_calibration_camera_velodyne)

## Links
[GitHub Repo](https://github.com/belenbarbed/FYP-VehicleSensorFusion)

[Google Drive Folder](https://drive.google.com/drive/folders/1wz1pCsgVdYGVe1YnCs0_oeLTOG-cAbBa?usp=sharing)

## Project Milestones

### Equipment
  - Lidar: Velodyne VLP-16 + mount
    - [Product page](https://velodynelidar.com/vlp-16-lite.html)
    - [User manual](https://velodynelidar.com/docs/manuals/63-9243%20REV%20D%20MANUAL,USERS,VLP-16.pdf)
    - [ROS integration instructions](http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16)
  - 4 Google Pixel 2 XL + gopro mounts
  - Mount to vehicle
    - Prototype myself
    - Circular & symmetric base => thick acrylic?
    - Aluminium extrusions to hold to car
    - Metal stand-offs to support base (4) and lidar (1)
    - 4 holes for gopro mounts (laser cut/drill?)
  - OR get made externally

### ROS
  - Install ROS
  - Make simple system:
    - Publisher 1: fetches cloudpoints from lidar, publishes them (topic: cloudpoints)
    - Publisher 2: fetches processed frames from server, publishes them (topic: frames)
    - Subscriber: subscribes to both
      - Interpolates info from where cars are in the frame with which cloudpoints correspond to said car
      - Calculates the approximate distance & angle to said car (mean/average?)
      - Exports each frame with the car(s) detected and their distance info

### Server (Recognition System)
  - Get app on phones to stream footage to server => research existing
  - Accepts connection from 4 clients (4 smartphones)
  - For each frame per client, recognises cars
  - Outputs frame with boundary boxes for each car
