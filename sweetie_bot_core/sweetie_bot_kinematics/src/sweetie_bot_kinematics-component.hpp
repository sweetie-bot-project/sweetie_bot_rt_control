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
#include <sweetie_bot_kinematics_msgs/typekit/CartesianState.h>

using namespace std;
using namespace RTT;
using namespace KDL;
//using namespace TRAC_IK;
//using namespace Logger;

class Sweetie_bot_kinematics : public RTT::TaskContext{
    struct LimbData
    {
	shared_ptr<Chain> chain;
	shared_ptr<ChainFkSolverPos_recursive> fk_solver;
	shared_ptr<TRAC_IK::TRAC_IK> ik_solver;
	shared_ptr<JntArray> seed;
    };

    boost::shared_ptr<RobotModel> robot_model_request_;
    InputPort<sensor_msgs::JointState> input_joint_state_;
    OutputPort<sensor_msgs::JointState> output_joint_state_;
    InputPort<sweetie_bot_kinematics_msgs::CartesianState> input_limbs_cartesian_;
    OutputPort<sweetie_bot_kinematics_msgs::CartesianState> output_limbs_cartesian_;
    vector<string> chain_names_;
    unordered_map<string,LimbData> limb_;
    //unordered_map<string,Chain*> chains_;
    //unordered_map<string,ChainFkSolverPos_recursive*> fk_solvers_;
    //unordered_map<string,TRAC_IK::TRAC_IK*> ik_solvers_;
    //unordered_map<string,JntArray*> joint_seed_;
  public:
    Sweetie_bot_kinematics(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
};
#endif
