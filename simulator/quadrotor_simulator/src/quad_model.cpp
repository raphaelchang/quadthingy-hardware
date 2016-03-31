#include <quadrotor_simulator/quad_model.h>

namespace gazebo
{
    void QuadModel::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model_ = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&QuadModel::OnUpdate, this, _1));

      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "quad_sim",
        ros::init_options::NoSigintHandler);
      }

      this->rosNode_.reset(new ros::NodeHandle("quad_sim"));
      ros::SubscribeOptions so = 
        ros::SubscribeOptions::create<geometry_msgs::Twist>(
                "/force",
                1,
                boost::bind(&QuadModel::ForceCb, this, _1),
                ros::VoidPtr(), &this->rosQueue_);
        this->rosSub_ = this->rosNode_->subscribe(so);

        this->rosQueueThread_ =
              std::thread(std::bind(&QuadModel::QueueThread, this));

        this->forces_ = math::Vector3(0,0,0);
        this->moments_ = math::Vector3(0,0,0);
    }

    void QuadModel::ForceCb(const geometry_msgs::Twist::ConstPtr& msg)
    {
        this->forces_ = math::Vector3(msg->linear.x, msg->linear.y, msg->linear.z);
        this->moments_ = math::Vector3(msg->angular.x, msg->angular.y, msg->angular.z);
    }

    void QuadModel::QueueThread()
    {
        static const double timeout = 0.01;
        while (this->rosNode_->ok())
        {
            this->rosQueue_.callAvailable(ros::WallDuration(timeout));
        }
    }

    // Called by the world update start event
    void QuadModel::OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      this->link_ = this->model_->GetLink();
      
      //publish forces and moments to model
      this->link_->AddRelativeForce(this->forces_);
      this->link_->AddRelativeTorque(this->moments_);
      
      //debug
     // math::Vector3 f = this->link_->GetRelativeForce();
     // ROS_INFO("X: %f, Y: %f, Z: %f", f[0], f[1], f[2]);
    }

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(QuadModel)
}
