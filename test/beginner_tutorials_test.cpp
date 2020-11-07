/* @file beginner_tutorials.cpp
 * @brief Test suite for the Beginner Tutorials Package
 *
 * @copyright [2020] Daniel M. Sahu [MIT]
 */

#include <ros/ros.h>
#include <gtest/gtest.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <std_msgs/String.h>

#include <atomic>

// Test that our transform broadcaster is working as expected.
TEST(BeginnerTutorials, TransformBroadcaster) {
  // instantiate tf listener / buffer
  tf2_ros::Buffer tfb;
  tf2_ros::TransformListener tfl(tfb);

  // check for expected transform
  ASSERT_TRUE(tfb.canTransform("world", "talk", ros::Time(0.0),
              ros::Duration(1.0)));

  // actually get the transform
  auto transform = tfb.lookupTransform("world", "talk", ros::Time(0.0));
  EXPECT_FLOAT_EQ(transform.transform.translation.x, 1.0);
}

// Test that our publisher is publishing
TEST(BeginnerTutorials, TopicPublisher) {
  // initialize a counter (to check number of callbacks)
  std::atomic<unsigned int> callback_count {0};

  // subscribe to our expected topic
  ros::NodeHandle nh;
  auto listener = nh.subscribe<std_msgs::String>(
    "chatter",
    1,
    [&callback_count] (const auto&) {
      ++callback_count;
    });

  // stop and spin for a little while, to give us time to collect a callback
  auto st = ros::Time::now();
  while (ros::Time::now() - st < ros::Duration(1.0))
    ros::spinOnce();

  // check that we've gotten an update transform
  EXPECT_NE(static_cast<unsigned int>(callback_count), 0);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "beginner_tutorials_test");
  ros::NodeHandle nh;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
