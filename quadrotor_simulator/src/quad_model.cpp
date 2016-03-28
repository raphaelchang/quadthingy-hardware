#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <ros/ros.h>

namespace gazebo
{
  class QuadModel : public ModelPlugin
  {

    // Pointer to the model
    private: physics::ModelPtr model;
    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    // Pointer to the link of the model    
    private: physics::LinkPtr link;
    
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&QuadModel::OnUpdate, this, _1));
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      this->link = model->GetLink();
      this->link->AddRelativeForce(math::Vector3(1,0,10));
      math::Vector3 f = this->link->GetRelativeForce();
      
      ROS_INFO("X: %f, Y: %f, Z: %f", f[0], f[1], f[2]);
      
      // Apply a small linear velocity to the model.
      //this->model->SetLinearVel(math::Vector3(.03, 0, 0));
    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(QuadModel)
}
