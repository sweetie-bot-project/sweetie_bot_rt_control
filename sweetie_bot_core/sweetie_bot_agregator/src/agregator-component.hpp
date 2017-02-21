#ifndef OROCOS_SWEETIE_BOT_AGREGATOR_COMPONENT_HPP
#define OROCOS_SWEETIE_BOT_AGREGATOR_COMPONENT_HPP

#include <rtt/RTT.hpp>

#include <kdl/chain.hpp>
#include <sweetie_bot_robot_model/robot_model-requester.hpp>

#include <sweetie_bot_logger/logger.hpp>

namespace sweetie_bot {
namespace motion {

class Agregator : public RTT::TaskContext{

    boost::shared_ptr<RobotModel> robot_model_;
    boost::shared_ptr<RobotModelInterface> robot_model_interface_;

    RTT::InputPort<sensor_msgs::JointState> input_port_joint_state_;
    RTT::OutputPort<sensor_msgs::JointState> output_port_joint_state_;

    sensor_msgs::JointState input_joint_state_;
    sensor_msgs::JointState output_joint_state_;

    std::vector<std::string> chain_names_;
    std::vector<std::string> joint_names_;

    // logging
#ifdef SWEETIEBOT_LOGGER
    sweetie_bot::logger::SWEETIEBOT_LOGGER log;
#else
    sweetie_bot::logger::LoggerLog4Cpp log;
#endif

  public:
    Agregator(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
};

} // namespace motion
} // namespace sweetie_bot
#endif
