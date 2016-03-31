#include <ros/ros.h>
#include <controller/Motors.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Geometry>

void motors_cb(const controller::Motors::ConstPtr& msg)
{
  //get quad charectaristics
  float d = msg->d;
  float kf = msg->kf;
  float kt = msg->kt;

  //get motor speeds (rad/s)
  const Eigen::Vector4d motor_speeds(
      msg->motor1,
      msg->motor2,
      msg->motor3,
      msg->motor4);

  //calculate forces and moments
  const Eigen::Vector4d motor_thrust = kf * motor_speeds.cwiseProduct(motor_speeds);
  Eigen::Matrix4d convert;
  convert << 1, 1, 1, 1,
             0, d, 0, -d,
             -d, 0, d, 0,
             kt, -kt, kt, -kt;
  
  const Eigen::Vector4d f_moments = convert * motor_thrust;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motors");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/motors", 1, motors_cb);


  ros::spin();

  return 0;
}
