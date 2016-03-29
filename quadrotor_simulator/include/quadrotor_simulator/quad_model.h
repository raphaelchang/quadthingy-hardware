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
        public:
            //QuadModel();
            //virtual ~QuadModel();
        
        protected:
            virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
            virtual void OnUpdate(const common::UpdateInfo & /*_info*/);

        private:
            physics::ModelPtr model_;
            event::ConnectionPtr updateConnection_;
            physics::LinkPtr link_;
    };
}
