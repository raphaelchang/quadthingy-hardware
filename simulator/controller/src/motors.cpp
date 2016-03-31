#include <ros/ros.h>
#include <controller/Motors.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Geometry>

void motors_cb(const controller::Motors::ConstPtr& msg)
{
  ROS_INFO("Got motors message!");

  //get motor speeds (rad/s)
  const Eigen::Vector4d motor_speeds(
      msg->motor1,
      msg->motor2,
      msg->motor3,
      msg->motor4);

  const Eigen::Vector4d motor_thrust = msg->kf * motor_speeds.cwiseProduct(motor_speeds);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motors");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/motors", 1, motors_cb);

  ros::spin();

  return 0;
}
