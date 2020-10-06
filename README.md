# motorosudp

## If you have not had the workspace yet, please create a workspace ("catkin_ws" is the workspace name):
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
$ source devel/setup.bash

## Change to the root of the Catkin workspace
$ cd $HOME/catkin_ws

## Clone the code:
$ git clone https://github.com/tapati0127/motorosudp.git

## check build dependencies. Note: this may install additional packages,
## depending on the software installed on the machine
$ rosdep update

## be sure to change 'kinetic' to whichever ROS release you are using
$ rosdep install --from-paths src/ --ignore-src --rosdistro kinetic

## build the workspace (using catkin_tools)
$ catkin build
