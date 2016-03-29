#include "ros/ros.h"
#include "controller/Motors.h"
#include "geometry_msgs/Twist.h"

void motors_cb(const controller::Motors::ConstPtr& msg)
{
    ROS_INFO("Got motors message!");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motors");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/motors", 1, motors_cb);

  ros::spin();

  return 0;
}
