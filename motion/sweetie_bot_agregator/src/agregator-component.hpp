#ifndef OROCOS_SWEETIE_BOT_AGREGATOR_COMPONENT_HPP
#define OROCOS_SWEETIE_BOT_AGREGATOR_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/os/Timer.hpp>

#include <kdl/chain.hpp>
#include <sweetie_bot_robot_model/robot_model-simple.hpp>

#include <sweetie_bot_orocos_misc/joint_state_check.hpp>

#include <sensor_msgs/typekit/JointState.h>

#include <sweetie_bot_logger/logger.hpp>

namespace sweetie_bot {
namespace motion {

class Agregator : public RTT::TaskContext
{
  protected:
	//COMPONENT INTERFACE
	// ports
    RTT::InputPort<sensor_msgs::JointState> input_port_joint_state_;
    RTT::OutputPort<sensor_msgs::JointState> output_port_joint_state_;
    RTT::InputPort<RTT::os::Timer::TimerId> sync_port_;
	// properties
	bool publish_on_timer_;
	bool publish_on_event_;
	// subservice
    boost::shared_ptr<RobotModel> robot_model_;

	// COMPONENT STATE
	// ports buffers
    sensor_msgs::JointState input_joint_state_;
    sensor_msgs::JointState output_joint_state_;
	// indexes of joints in full pose
    std::map< std::string, int > joint_index_;

    // logging
#ifdef SWEETIEBOT_LOGGER
    sweetie_bot::logger::SWEETIEBOT_LOGGER log;
#else
    sweetie_bot::logger::LoggerRTT log;
#endif
  protected:
    const unsigned int max_requests_per_cycle = 10;

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
