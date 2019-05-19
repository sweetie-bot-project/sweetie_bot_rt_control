#ifndef OROCOS_SWEETIE_BOT_KINEMATICS_COMPONENT_HPP
#define OROCOS_SWEETIE_BOT_KINEMATICS_COMPONENT_HPP

#include <unordered_map>

#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>

#include <rtt/RTT.hpp>
#include <rtt/Logger.hpp>

#include <trac_ik/trac_ik.hpp>
#include <orocos/sensor_msgs/typekit/JointState.h>

#include <sweetie_bot_robot_model/robot_model.hpp>
#include <sweetie_bot_kinematics_msgs/typekit/RigidBodyState.h>

#include <sweetie_bot_logger/logger.hpp>

namespace sweetie_bot {
namespace motion {

class KinematicsFwd : public RTT::TaskContext
{
	protected:
		// TODO use Tree solver ?
		// TODO It needs Tree kdl model.
		// TODO Jacobian calculation.
		// Support ащк base connected joints?
		struct KinematicChainData {
			std::string name; /**< Kinematic chain name */
			int index_begin; /**< Index of first joint in chain */
			std::unique_ptr<KDL::Chain> chain; /**< Kinematic chain. KDL 1.4 FKSolvers store reference to KDL::Chain so Chain object must not change memory location. */
			std::unique_ptr<KDL::ChainFkSolverVel_recursive> fk_vel_solver; /**< FK solver */
			KDL::JntArrayVel jnt_array_vel; /**< buffer */
		};

	protected: 
		// COMPONENT INTERFACE 
		// ports
		RTT::InputPort<sensor_msgs::JointState> in_joints_port;
		RTT::OutputPort<sweetie_bot_kinematics_msgs::RigidBodyState> out_limbs_port;
		// properties
		std::vector<std::string> chain_names;
		bool virtual_links;
		// subservices
		RobotModel * robot_model;

		// COMPONENT STATE
		// buffers
		sensor_msgs::JointState joints;
		sweetie_bot_kinematics_msgs::RigidBodyState limbs;
		// struct array with solvers and information about chains
		std::vector<KinematicChainData> chain_data;
		int n_joints;

		// logging
#ifdef SWEETIEBOT_LOGGER
		sweetie_bot::logger::SWEETIEBOT_LOGGER log;
#else
		sweetie_bot::logger::LoggerRTT log;
#endif

	public:
		KinematicsFwd(std::string const& name);
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
};

} // namespace motion
} // namespace sweetie_bot

#endif
