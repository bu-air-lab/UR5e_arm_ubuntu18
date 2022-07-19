# This page records how to manipulate UR5e in both real and simulated worlds


## Basics of UR5e (real robot)
If this is your first time, please check the [Wiki page](https://github.com/bu-air-lab/UR5e_arm/wiki) of this repository to learn how to work with the arm


## Install ROS (melodic) on Ubuntu 18.04
Detailed instruction can be found at http://wiki.ros.org/melodic/Installation/Ubuntu
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-melodic-desktop-full
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
source /opt/ros/melodic/setup.bash
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt install python-rosdep
sudo rosdep init
rosdep update
```

## Set Up Environment on PC
### Create a catkin workspace
```
mkdir -p ~/catkin_ws/src
cd catkin_ws
```

### Add the local environment path
Note, replace **'/home/yan/catkin_ws/devel/setup.bash'** with your own path
```
echo "source /home/yan/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Install required libraries
```
sudo apt-get install ros-melodic-moveit && sudo apt-get install ros-melodic-ros-control ros-melodic-ros-controllers && sudo apt-get install ros-melodic-industrial-msgs && sudo apt-get install ros-melodic-soem
wstool init src https://raw.githubusercontent.com/yding25/UR5e_arm_ubuntu18/master/rosinstall/melodic.rosinstall
sudo apt update -qq
rosdep update
rosdep install --from-paths src --ignore-src -y
```

### Compile
``` 
catkin_make
```

## Manipulate UR5e in Gazebo using Moveit and Rviz
Open a new terminal, and input
```
roslaunch ur_gazebo ur5e_bringup.launch
```
Open a new terminal, and input
```
roslaunch ur5e_moveit_config ur5e_moveit_planning_execution.launch sim:=true
```
Open a new terminal, and input
```
roslaunch ur5e_moveit_config moveit_rviz.launch rviz_config:=$(rospack find ur5_moveit_config)/launch/moveit.rviz
```
Now, you can manipulate UR5e arm usuing RVIZ.


## Manipulate UR5e in real world
### Method 1: execute commands one by one
Open a new terminal, and input
```
roslaunch ur_robot_driver ur5e_bringup.launch limited:=true robot_ip:=10.32.134.142
```
Note, replace **10.32.134.142** with your robot ip

On the teaching board, load **external_control_yan.urp**, and **run** it. You will see the following information on the previous terminal:
```
[ INFO] [1615715907.527568874]: Robot requested program
[ INFO] [1615715907.527840385]: Sent program to robot
[ INFO] [1615715907.564855461]: Robot ready to receive control commands.
```

After the information exists, open a new terminal, and input
```
roslaunch ur5e_moveit_config ur5e_moveit_planning_execution.launch limited:=true
```
Open a new terminal, and input
```
roslaunch ur5e_moveit_config moveit_rviz.launch rviz_config:=$(rospack find ur5_moveit_config)/launch/moveit.rviz
```
Now, you can manipulate UR5e arm usuing RVIZ.

### Method 2: execute lauch file
```
roslaunch ur_move_test ur5_move_test.launch
```
Note, replace **10.32.134.142** with your robot ip in the launch file **"/home/yan/catkin_ws/src/ur_move_test/launch/ur5_move_test.launch"** 


### Manipulate UR5e using Python
```
python /home/yan/catkin_ws/src/ur_move_test/scripts/ur_move_test_node.py
```
Note, replace **'/home/yan/catkin_ws/src/ur_move_test/scripts/'** with your own path


## Manipulate Gripper (Robotiq Hand-E)
```
sudo usermod -a -G dialout $USER 
dmesg | grep tty 
sudo chmod 777  /dev/ttyACM0 
rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyACM0
rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleController.py 
```
