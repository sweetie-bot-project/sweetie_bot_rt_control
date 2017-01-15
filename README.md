A repository for the SweetieBot robot project. Its aim is to create a robot 
that resembles Sweetie Bot from MLP fandom works (no, it won't be self aware 
any time soon). The project is being built using ROS.

```
sudo apt-get install qt5-default ros-indigo-trac-ik ros-indigo-rtt-roscomm ros-indigo-rtt-rosparam ros-indigo-rtt-std-msgs ros-indigo-rtt-rosgraph-msgs ros-indigo-rtt-common-msgs ros-indigo-rtt-ros-integration
```
```
mkdir -p ~/ros/sw/src
cd ~/ros/sw/src
git clone git@gitlab.com:sweetiebot/sweetie_bot.git
cd ..
catkin_make -j1
```
