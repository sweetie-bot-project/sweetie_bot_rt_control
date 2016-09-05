#ifndef OROCOS_SWEETIE_BOT_KINEMATICS_COMPONENT_HPP
#define OROCOS_SWEETIE_BOT_KINEMATICS_COMPONENT_HPP

#include <unordered_map>

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <rtt/RTT.hpp>
#include <rtt/Logger.hpp>

#include <trac_ik/trac_ik.hpp>

#include <sweetie_bot_robot_model/sweetie_bot_robot_model-requester.hpp>
#include <sweetie_bot_kinematics_msgs/typekit/LimbState.h>

using namespace std;
using namespace RTT;
using namespace KDL;

class Sweetie_bot_kinematics : public RTT::TaskContext{
    struct LimbData
    {
	shared_ptr<Chain> chain;
	shared_ptr<vector<string>> joints;
	shared_ptr<ChainFkSolverPos_recursive> fk_solver;
	shared_ptr<TRAC_IK::TRAC_IK> ik_solver;
	shared_ptr<JntArray> seed;
    };

    boost::shared_ptr<RobotModel> robot_model_;
    boost::shared_ptr<RobotModelInterface> robot_model_interface_;

    InputPort<sensor_msgs::JointState> input_port_joints_seed_;
    InputPort<sensor_msgs::JointState> input_port_joints_;
    OutputPort<sensor_msgs::JointState> output_port_joints_;
    InputPort<sweetie_bot_kinematics_msgs::LimbState> input_port_limbs_;
    OutputPort<sweetie_bot_kinematics_msgs::LimbState> output_port_limbs_;

    vector<string> chain_names_;
    vector<string> joint_names_;
    unordered_map<string,LimbData> limb_;
  public:
    Sweetie_bot_kinematics(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
};
#endif
