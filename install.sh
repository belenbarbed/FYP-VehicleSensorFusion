# Install ROS melodic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

sudo apt-get update
sudo apt install ros-melodic-desktop-full

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get install python-catkin-tools

sudo rosdep init
rosdep update

echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential

# Make ROS workspace
mkdir -p ros_ws/src

cd ros_ws

catkin_make
catkin_make install

# Velodyne lidar stuff
sudo apt-get install ros-melodic-velodyne

cd src
git submodule add https://github.com/ros-drivers/velodyne

rosdep install --from-paths src --ignore-src --rosdistro meodic -y

cd ../
catkin_make
