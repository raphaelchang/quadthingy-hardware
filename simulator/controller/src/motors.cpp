#include <ros/ros.h>
#include <controller/Motors.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Geometry>

namespace motors
{
  class Node {
    public:
      explicit Node(const ros::NodeHandle& pnh);
      void motors_cb(const controller::Motors::ConstPtr& msg);

    private:
      ros::NodeHandle pnh_;
      ros::Subscriber motors_sub_;
      ros::Publisher force_pub_;

  }; //class Node

  Node::Node(const ros::NodeHandle& pnh) : pnh_(pnh)
  {
    motors_sub_ = pnh_.subscribe("motor_speeds", 1, &Node::motors_cb, this);
    force_pub_ = pnh_.advertise<geometry_msgs::Twist>("force", 1); 
  }

  void Node::motors_cb(const controller::Motors::ConstPtr& msg)
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

    //publish message
    geometry_msgs::Twist out;
    
    out.linear.z = f_moments[0];
    out.angular.x = f_moments[1];
    out.angular.y = f_moments[2];
    out.angular.z = f_moments[3];

    force_pub_.publish(out);
  }
} //namespace motors

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motors");

  ros::NodeHandle pnh("~");

  motors::Node node(pnh);

  ros::spin();

  return 0;
}
