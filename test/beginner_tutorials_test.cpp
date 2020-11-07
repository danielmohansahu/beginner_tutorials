#include <ros/ros.h>
#include <gtest/gtest.h>

TEST(DummyTest, DummyTestPass) {
  EXPECT_TRUE(true);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "beginner_tutorials_test");
  ros::NodeHandle nh;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
