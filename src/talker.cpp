/* @file talker.cpp
 * @brief An example ROS Publisher
 *
 * @copyright [2020] Daniel M. Sahu [MIT]
 */

#include <atomic>
#include <limits>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * Initialize our count of how many messages we've sent.
   */
  std::atomic<int> count {0};

  /**
   * Define a service that allows us to reset our message count.
   */
  ros::ServiceServer service = n.advertiseService(
    "reset_count",
    boost::function<bool(std_srvs::Empty::Request&, std_srvs::Empty::Response&)>(
      [&count](const auto&, const auto&)
      {
        ROS_INFO_STREAM_NAMED("talker", "Resetting count from " << count << " to 0.");
        count = 0;
        return true;
      }
    )
  );

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ROS_DEBUG_NAMED("talker", "Publisher advertised.");

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  while (ros::ok()) {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    ROS_DEBUG_THROTTLE_NAMED(1.0, "talker", "Executing callback loop.");

    std_msgs::String msg;
    msg.data = "Do you think I talk too much? ["
               + std::to_string(count) + "/"
               + std::to_string(std::numeric_limits<int>::max()) + "]";
    ROS_INFO_STREAM(msg.data);

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;

    // make sure we haven't been running for too long
    if (count == std::numeric_limits<int>::max()) {
      ROS_FATAL_NAMED("talker", "Runtime execution went on too long; about to hit integer overflow.");
      return 1;
    }
  }

  ROS_INFO("Talker node initialized.");

  return 0;
}
