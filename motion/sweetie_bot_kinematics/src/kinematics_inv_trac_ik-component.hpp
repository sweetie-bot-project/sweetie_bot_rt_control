#ifndef OROCOS_SWEETIE_BOT_KINEMATICS_INV_TRAC_IK_COMPONENT_HPP
#define OROCOS_SWEETIE_BOT_KINEMATICS_INV_TRAC_IK_COMPONENT_HPP

#include <kdl/chain.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <trac_ik/trac_ik.hpp>

#include <rtt/RTT.hpp>
#include <rtt/Logger.hpp>

#include <sweetie_bot_logger/logger.hpp>
#include <sweetie_bot_robot_model/robot_model.hpp>

#include <sensor_msgs/typekit/JointState.h>
#include <sweetie_bot_kinematics_msgs/typekit/RigidBodyState.h>


namespace sweetie_bot {
namespace motion {

class KinematicsInvTracIK : public RTT::TaskContext
{
	protected:
		struct KinematicChainData {
			std::string name; /**< Kinematic chain name */
			std::vector<string> joint_names; /**< Names of joint. */
			int index_begin; /**< Index of first joint in chain */
			int size; /**< Kinematic chain length. */
			shared_ptr<TRAC_IK::TRAC_IK> ik_solver; /**< IK  velocity solver */
			shared_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver; /**< IK  velocity solver */
			KDL::JntArray jnt_array_pose; /**< buffer */
			KDL::JntArray jnt_array_vel; /**< buffer */
			KDL::JntArray jnt_array_seed_pose; /**< initial approximation for solution */
		};

	protected:

		// COMPONENT INTERFACE 
		// ports
		RTT::InputPort<sensor_msgs::JointState> in_joints_seed_port_;
		RTT::OutputPort<sensor_msgs::JointState> out_joints_port_;
		RTT::InputPort<sweetie_bot_kinematics_msgs::RigidBodyState> in_limbs_port_;
		// properties
		std::vector<std::string> chain_names_;
		int max_iterations_;
		double eps_vel_;
		double eps_pos_;
		double timeout_;
		bool use_ik_pose_as_new_seed_;
		bool zero_vel_at_singularity_;
		// subservices
		RobotModel * robot_model_;

		// COMPONENT STATE
		// buffers
		sensor_msgs::JointState joints_;
		sweetie_bot_kinematics_msgs::RigidBodyState limbs_;
		// struct array with solvers and information about chains
		std::vector<KinematicChainData> chain_data_;
		int n_joints_fullpose_;

		// logging
#ifdef SWEETIEBOT_LOGGER
		sweetie_bot::logger::SWEETIEBOT_LOGGER log;
#else
		sweetie_bot::logger::LoggerRTT log;
#endif
	public:
		// operations
		bool poseToJointState(const sweetie_bot_kinematics_msgs::RigidBodyState& in, sensor_msgs::JointState& out); 

	public:

		KinematicsInvTracIK(std::string const& name);

		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
};

} // namespace motion
} // namespace sweetie_bot

#endif
