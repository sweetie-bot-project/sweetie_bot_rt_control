#ifndef OROCOS_SWEETIE_BOT_AGGREGATOR_COMPONENT_HPP
#define OROCOS_SWEETIE_BOT_AGGREGATOR_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/os/Timer.hpp>

#include <sensor_msgs/typekit/JointState.h>

#include <sweetie_bot_logger/logger.hpp>
#include <sweetie_bot_robot_model/robot_model.hpp>

#include <sweetie_bot_kinematics_msgs/typekit/SupportState.h>

namespace sweetie_bot {
namespace motion {

class Aggregator : public RTT::TaskContext
{
  protected:
	//COMPONENT INTERFACE
	// ports
    RTT::InputPort<sensor_msgs::JointState> input_port_joint_state_;
    RTT::InputPort<sweetie_bot_kinematics_msgs::SupportState> input_port_support_state_;
    RTT::InputPort<RTT::os::Timer::TimerId> sync_port_;
    RTT::OutputPort<sensor_msgs::JointState> output_port_joint_state_;
    RTT::OutputPort<sweetie_bot_kinematics_msgs::SupportState> output_port_support_state_;
	// properties
	bool publish_on_timer_;
	bool publish_on_event_;
	// subservice
    boost::shared_ptr<RobotModel> robot_model_;

	// COMPONENT STATE
	std::vector<std::string> chain_default_contacts_;
	// ports buffers
    sensor_msgs::JointState input_joint_state_;
    sensor_msgs::JointState output_joint_state_;
	sweetie_bot_kinematics_msgs::SupportState input_support_state_;
	sweetie_bot_kinematics_msgs::SupportState output_support_state_;
	// indexes of joints in full pose
    std::map< std::string, int > joint_index_;
    std::map< std::string, int > chain_index_;

    // logging
#ifdef SWEETIEBOT_LOGGER
    sweetie_bot::logger::SWEETIEBOT_LOGGER log;
#else
    sweetie_bot::logger::LoggerRTT log;
#endif
  protected:
    const unsigned int max_requests_per_cycle = 10;

  public:
	// OPERATIONS
	bool setSupportState(std::vector<string> limbs);

    Aggregator(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
};

} // namespace motion
} // namespace sweetie_bot
#endif
