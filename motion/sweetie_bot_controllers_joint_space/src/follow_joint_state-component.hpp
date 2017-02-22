#ifndef  FOLLOW_JOINT_STATE_COMPONENT_HPP
#define  FOLLOW_JOINT_STATE_COMPONENT_HPP

#include <vector>
#include <string>
#include <unordered_map>

#include <rtt/Component.hpp>
#include <rtt/os/Timer.hpp>

#include <std_srvs/SetBool.h>
#include <orocos/sensor_msgs/typekit/JointState.h>

#include <sweetie_bot_logger/logger.hpp>
#include <sweetie_bot_resource_control/resource_client.hpp>
#include <sweetie_bot_robot_model/sweetie_bot_robot_model-requester.hpp>

namespace sweetie_bot {
namespace motion {
namespace controller {


class FollowJointState : public RTT::TaskContext
{
	protected:
		// COMPONENT INTERFACE
		//
		// PORTS: input
		RTT::InputPort<RTT::os::Timer::TimerId> sync_port;
		RTT::InputPort<sensor_msgs::JointState> in_joints_port;
		RTT::InputPort<sensor_msgs::JointState> in_joints_ref_port;
		// PORTS: output
		RTT::OutputPort<sensor_msgs::JointState> out_joints_port;
		// PROPERTIES
		std::vector<std::string> controlled_chains;
	protected:
		// OPERATIONS: provides
		bool rosSetOperational(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp);
		// OPERATIONS: requires
		// SERVICES: provides
		// SERVICES: required
		sweetie_bot::motion::RobotModel robot_model; // joints list, kinematics chains access
		// SERVICES: internal interface
		sweetie_bot::motion::ResourceClientInterface * resource_client; // resource client
		// sweetie_bot::filters::TransientJointStateInterface * filter; // trajectory smoother

	protected:
		struct JointIndex {
			unsigned int index_fullpose; // joint index in full pose vector
			unsigned int index; // joint index in controlled JointState vector
			JointIndex() {}
			JointIndex(unsigned int _index_fullpose, unsigned int _index) : index_fullpose(_index_fullpose), index(_index) {}
		};
		typedef std::unordered_map<std::string, JointIndex> JointIndexes;

	protected:
		// COMPONENT STATE
		JointIndexes controlled_joints; // joint indexes cache
		// ports buffers
		sensor_msgs::JointState ref_pose_unsorted; // buffer for input port in_joints_ref_port
		sensor_msgs::JointState actual_fullpose; // buffer for input port in_joints_port
		sensor_msgs::JointState actual_pose; // controlled joints actual position
		sensor_msgs::JointState ref_pose; // controlled joints ref position
		
#ifdef SWEETIEBOT_LOGGER
		sweetie_bot::logger::SWEETIEBOT_LOGGER log;
#else
		sweetie_bot::logger::LoggerRTT log;
#endif

	public:
		FollowJointState(std::string const& name);

		bool resourceChangedHook();

		bool configureHook(); 
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
};

} // namespace controller
} // namespace motion
} // namespace sweetie_bot

#endif  /*FOLLOW_JOINT_STATE_COMPONENT_HPP*/
