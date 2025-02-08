#ifndef OROCOS_SWEETIE_BOT_KINEMATICS_INV_TRAC_IK_COMPONENT_HPP
#define OROCOS_SWEETIE_BOT_KINEMATICS_INV_TRAC_IK_COMPONENT_HPP

#include <kdl/chain.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <rtt/RTT.hpp>
#include <rtt/Logger.hpp>

#include <sweetie_bot_logger/logger.hpp>
#include <sweetie_bot_robot_model/robot_model.hpp>

#include <sensor_msgs/typekit/JointState.h>
#include <sweetie_bot_kinematics_msgs/typekit/RigidBodyState.h>

#include <sweetie_bot_kinematics/solver_ik.hpp>

namespace sweetie_bot {
namespace motion {

class KinematicsInv : public RTT::TaskContext
{
	protected:
		enum { NO_SOLUTION = -1, TOLERANCE_VIOLATION_FLAG = 1, LOCALITY_VIOLATION_FLAG = 2 };

		struct KinematicChainData {
			std::string name; /**< Kinematic chain name */
			std::vector<std::string> joint_names; /**< Names of joint. */
			std::vector<int> joint_induces; /**< Induces of joints in chain */
			int size; /**< Kinematic chain length. */
			int size_real; /**< Kinematic chain length without fictive joints */
			std::unique_ptr<KDL::Chain> chain; /**< Kinematic chain. KDL 1.4 FKSolvers store reference to KDL::Chain so Chain object must not change memory location. */ //TODO: remove size field?
			std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver; /**< FK  pose solver */
			std::unique_ptr<SolverIKInterface> ik_solver; /**< IK position solver */
			std::unique_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver; /**< IK  velocity solver */
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
		bool use_ik_pose_as_new_seed_;
		bool zero_vel_at_singularity_;
		double max_joint_velocity_;
		double period_;
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
	protected:
		int poseToJointState_impl(const sweetie_bot_kinematics_msgs::RigidBodyState& in, sensor_msgs::JointState& out);
		std::unique_ptr<SolverIKInterface> getIKSolver(const std::string& name, const KDL::Chain& chain, const std::vector<SolverIKFactoryInterface *>& solver_ik_factories);

		// operations
		int poseToJointState(const sweetie_bot_kinematics_msgs::RigidBodyState& in, sensor_msgs::JointState& out);
		bool poseToJointStatePublish(const sweetie_bot_kinematics_msgs::RigidBodyState& in, int approx_mode);

	public:

		KinematicsInv(const std::string& name);

		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
};

} // namespace motion
} // namespace sweetie_bot

#endif
