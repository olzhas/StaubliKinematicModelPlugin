#ifndef GAZEBO_STAUBLI_KINEMATIC_MODEL_PLUGIN_HH
#define GAZEBO_STAUBLI_KINEMATIC_MODEL_PLUGIN_HH

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
    class GAZEBO_VISIBLE StaubliKinematicModelPlugin : public ModelPlugin
    {
        public: StaubliKinematicModelPlugin();

        public: virtual ~StaubliKinematicModelPlugin();

        public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

        private: void UpdateStates(const common::UpdateInfo &_info);

        private: physics::WorldPtr world;
        private: physics::ModelPtr model;
        private: physics::JointPtr joint;

        private: boost::mutex update_mutex;

        private: event::ConnectionPtr updateConnection;
    };
}

#endif
