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
    }

    // Called by the world update start event
    void QuadModel::OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      this->link_ = this->model_->GetLink();
      this->link_->AddRelativeForce(math::Vector3(1,0,10));
      math::Vector3 f = this->link_->GetRelativeForce();
      
      ROS_INFO("X: %f, Y: %f, Z: %f", f[0], f[1], f[2]);
    }

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(QuadModel)
}
