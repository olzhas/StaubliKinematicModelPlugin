// based on JointTrajectoryPlugin.cc

#include "StaubliKinematicModelPlugin.hh"

namespace gazebo
{
    //////////////////////////////////////////////////////////
    StaubliKinematicModelPlugin::StaubliKinematicModelPlugin()
    {
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
            NULL, this);
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

        joint_position_map["TX90XLHB::joint1"] = cos(cur_time.Double());
        joint_position_map["TX90XLHB::joint2"] = sin(cur_time.Double());
        joint_position_map["TX90XLHB::joint3"] = -sin(cur_time.Double());
        joint_position_map["TX90XLHB::joint4"] = -cos(cur_time.Double());
        joint_position_map["TX90XLHB::joint5"] = cos(cur_time.Double());
        joint_position_map["TX90XLHB::joint6"] = sin(cur_time.Double());
        this->model->SetJointPositions(joint_position_map);

        this->world->SetPaused(is_paused);
    }

    //////////////////////////////////////////////////////////
    void StaubliKinematicModelPlugin::setStates()
    {

    }
    GZ_REGISTER_MODEL_PLUGIN(StaubliKinematicModelPlugin)
}
