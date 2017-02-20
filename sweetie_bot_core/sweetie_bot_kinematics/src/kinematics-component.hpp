#ifndef OROCOS_SWEETIE_BOT_KINEMATICS_COMPONENT_HPP
#define OROCOS_SWEETIE_BOT_KINEMATICS_COMPONENT_HPP

#include <unordered_map>

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <rtt/RTT.hpp>
#include <rtt/Logger.hpp>

#include <trac_ik/trac_ik.hpp>

#include <sweetie_bot_robot_model/robot_model-requester.hpp>
#include <sweetie_bot_kinematics_msgs/typekit/LimbState.h>

#include <sweetie_bot_logger/logger.hpp>

namespace sweetie_bot {
namespace motion {

class Kinematics : public RTT::TaskContext{
    struct LimbData
    {
	shared_ptr<KDL::Chain> chain;
	shared_ptr<std::vector<std::string>> joints;
	shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
	shared_ptr<TRAC_IK::TRAC_IK> ik_solver;
	shared_ptr<KDL::JntArray> seed;
    };

    boost::shared_ptr<RobotModel> robot_model_;
    boost::shared_ptr<RobotModelInterface> robot_model_interface_;

    InputPort<sensor_msgs::JointState> input_port_joints_seed_;
    InputPort<sensor_msgs::JointState> input_port_joints_;
    OutputPort<sensor_msgs::JointState> output_port_joints_;
    InputPort<sweetie_bot_kinematics_msgs::LimbState> input_port_limbs_;
    OutputPort<sweetie_bot_kinematics_msgs::LimbState> output_port_limbs_;

    sensor_msgs::JointState input_joint_seed_;
    sensor_msgs::JointState input_joint_state_;
    sensor_msgs::JointState output_joint_state_;
    sweetie_bot_kinematics_msgs::LimbState input_limb_state_;
    sweetie_bot_kinematics_msgs::LimbState output_limb_state_;

    std::vector<std::string> chain_names_;
    std::vector<std::string> joint_names_;
    std::unordered_map<std::string,LimbData> limb_;

    // logging
#ifdef SWEETIEBOT_LOGGER
    sweetie_bot::logger::SWEETIEBOT_LOGGER log;
#else
    sweetie_bot::logger::LoggerLog4Cpp log;
#endif

  public:
    Kinematics(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
};

} // namespace motion
} // namespace sweetie_bot

#endif
