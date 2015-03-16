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
#include "gazebo/gazebo.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/math/gzmath.hh"

namespace gazebo
{
    class GAZEBO_VISIBLE StaubliKinematicModelPlugin : public ModelPlugin
    {
        public: StaubliKinematicModelPlugin();

        public: virtual ~StaubliKinematicModelPlugin();

        public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

        private: void UpdateStates(const common::UpdateInfo &_info);
        private: void setStates();

        private: physics::WorldPtr world;
        private: physics::ModelPtr model;
        private: physics::JointPtr joint;

        private: boost::mutex update_mutex;

        private: event::ConnectionPtr updateConnection;

        /// \brief Node used to establish communication with gzserver.
        private: transport::NodePtr node;

        /// \brief Publisher to world statistics messages.
        private: transport::SubscriberPtr stateSub;

        private: boost::mutex mutex;
    };
}

#endif
