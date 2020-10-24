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

To run the listener and talker several terminals are required (no launch file is provided):

```bash
# terminal #1
roscore

# terminal #2
source ~/catkin_ws/devel/setup.bash
rosrun beginner_tutorials listener

# terminal #3
source ~/catkin_ws/devel/setup.bash
rosrun beginner_tutorials talker
```

Alternatively, if you don't mind some mixing the nodes' output, the following will run everything in one terminal.

```bash
source ~/catkin_ws/devel/setup.bash
roscore &
rosrun beginner_tutorials listener &
rosrun beginner_tutorials talker
```


