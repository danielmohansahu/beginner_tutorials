# beginner_tutorials

This repository is a ROS Package following the [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials).

## Dependencies

This package was written and tested against ROS Melodic on Ubuntu 18.04 using a C++11 enabled compiler.
It should also function properly with most modern versions of ROS1.

Please see [the ROS documentation](http://wiki.ros.org/melodic/Installation) on how to install ROS Melodic.

## Installation / Build Instructions

Building this package from source requires a catkin workspace. The following full installation 
instructions are provided for reference:

```bash
# create catkin workspace
mkdir ~/catkin_ws/src -p

# clone repository / package
pushd ~/catkin_ws/src
git clone https://github.com/danielmohansahu/beginner_tutorials
popd

# build
catkin_make
```

## Run Instructions

To run the listener and talker there is an example launch file.

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch beginner_tutorials conversation.launch
```

Output should appear showing the listener is receiving messages. If you'd like to reset the message counter, call the following service:

```bash
rosservice call /conversation/reset_count "{}"
```

