// based on JointTrajectoryPlugin.cc

#include "StaubliKinematicModelPlugin.hh"
#include "staubli_joint_states.pb.h"

namespace gazebo
{
    //////////////////////////////////////////////////////////
    StaubliKinematicModelPlugin::StaubliKinematicModelPlugin()
    {
        for (int i = 0; i < numJoints; ++i) {
            jointStates[i] = 0;
        }
    }

    //////////////////////////////////////////////////////////
    StaubliKinematicModelPlugin::~StaubliKinematicModelPlugin()
    {
        event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
    }

    //////////////////////////////////////////////////////////
    void StaubliKinematicModelPlugin::Load( physics::ModelPtr _parent,
                                            sdf::ElementPtr _sdf)
    {
        // Get the world name.
        this->model = _parent;
        this->world = this->model->GetWorld();

        // this->world->GetPhysicsEngine()->SetGravity(math::Vector3(0,0,0));

        for(physics::Joint_V::const_iterator j = this->model->GetJoints().begin();
            j != this->model->GetJoints().end(); ++j)
            (*j)->SetPosition(0, 0);

        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    boost::bind(&StaubliKinematicModelPlugin::UpdateStates, this, _1));

        // Create a node for transportation
        this->node = transport::NodePtr(new transport::Node());
        this->node->Init();
        this->stateSub = this->node->Subscribe("~/staubli_joint_states",
                                               &StaubliKinematicModelPlugin::setStatesCallback, this);
    }

    //////////////////////////////////////////////////////////

    void StaubliKinematicModelPlugin::UpdateStates(const common::UpdateInfo &_info)
    {
        common::Time cur_time = this->world->GetSimTime();

        // for (physics::Joint_V::const_iterator j = this->model->GetJoints().begin();
        //                       j != this->model->GetJoints().end(); ++j)
        //   gzerr << cur_time << " " << (*j)->GetScopedName() << "\n";

        bool is_paused = this->world->IsPaused();
        if (!is_paused) this->world->SetPaused(true);

        std::map<std::string, double> joint_position_map;
        mutex.lock();
        joint_position_map["TX90XLHB::joint1"] = (double) jointStates[0] / 180.0 * M_PI;
        joint_position_map["TX90XLHB::joint2"] = (double) jointStates[1] / 180.0 * M_PI;
        joint_position_map["TX90XLHB::joint3"] = (double) jointStates[2] / 180.0 * M_PI;
        joint_position_map["TX90XLHB::joint4"] = (double) jointStates[3] / 180.0 * M_PI;
        joint_position_map["TX90XLHB::joint5"] = (double) jointStates[4] / 180.0 * M_PI;
        joint_position_map["TX90XLHB::joint6"] = (double) jointStates[5] / 180.0 * M_PI;
        mutex.unlock();
        this->model->SetJointPositions(joint_position_map);

        this->world->SetPaused(is_paused);
    }

    //////////////////////////////////////////////////////////
    void StaubliKinematicModelPlugin::setStatesCallback(StaubliJointStatesPtr &msg)
    {
        mutex.lock();

        jointStates[0] = msg->joint1();
        jointStates[1] = msg->joint2();
        jointStates[2] = msg->joint3();
        jointStates[3] = msg->joint4();
        jointStates[4] = msg->joint5();
        jointStates[5] = msg->joint6();

        mutex.unlock();
    }
    GZ_REGISTER_MODEL_PLUGIN(StaubliKinematicModelPlugin)
}
