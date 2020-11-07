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
roslaunch beginner_tutorials conversation.launch namespace:=conversation
```

Output should appear showing the listener is receiving messages. If you'd like to reset the message counter, call the following service:

```bash
rosservice call /conversation/reset_count "{}"
```

## Bag Inspection

The default launch file also launches a `rosbag record -a` session that records all topics. This can be disabled by 
passing the `bag:=false` to the launch. These bags will be saved to your ROS root directory (probably `~/.ros`).

To test if a bag is working properly you can run it against the listener node, e.g.

```bash
# terminal 1, roscore
roscore
```

```bash
# terminal 2, listener
source devel/setup.bash
rosrun beginner_tutorials listener
```

```bash
# terminal 3, bag playback
# note that we must remap topics, since the default listener launch prepends a namespace and modifies the topic
rosbag play {PATH_TO_BAG} /conversation/chatter:=listener_topic
```

## TF Introspection Instructions

The talker node will advertise a static TF transform on the `tf_static` topic. To introspect it avail yourself of tf's transform introspection tools:

```bash
# to view all currently advertised transforms
rosrun tf tf_monitor
# to output all currently advertised transforms to a PDF
rosrun tf view_frames
# to echo and inspect our specific transforms
rosrun tf tf_echo world talk
```

## Test Instructions

To evaluate our system tests execute the following command.

```bash
catkin_make run_tests
```

You should see output similar to the following:
```bash
[ROSTEST]-----------------------------------------------------------------------

[beginner_tutorials.rosunit-beginner_tutorials_test/TransformBroadcaster][passed]
[beginner_tutorials.rosunit-beginner_tutorials_test/TopicPublisher][passed]

SUMMARY
 * RESULT: SUCCESS
 * TESTS: 2
 * ERRORS: 0
 * FAILURES: 0
```