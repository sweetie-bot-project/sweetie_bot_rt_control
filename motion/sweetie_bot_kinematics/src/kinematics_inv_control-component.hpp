#ifndef  KINEMATICS_INV_CONTROL_COMPONENT_HPP
#define  KINEMATICS_INV_CONTROL_COMPONENT_HPP


#include <rtt/RTT.hpp>
#include <rtt/Logger.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <Eigen/Dense>
#include <Eigen/SVD>

#include <sweetie_bot_logger/logger.hpp>
#include <sweetie_bot_robot_model/robot_model.hpp>

#include <sensor_msgs/typekit/JointState.h>
#include <sweetie_bot_kinematics_msgs/typekit/RigidBodyState.h>



namespace sweetie_bot {
namespace motion {


class KinematicsInvControl : public RTT::TaskContext
{
	protected:
		enum { NO_SOLUTION = -1, TOLERANCE_VIOLATION_FLAG = 1, LOCALITY_VIOLATION_FLAG = 2 };

		// represent one step of IK control loop
		class IKControlSolver 
		{
			private: 
				// solvers
				KDL::Chain chain_; /**< Kinematic chain. KDL 1.4 FKSolvers store reference to KDL::Chain so Chain object must not change memory location. */ //TODO: remove size field?
				KDL::ChainFkSolverPos_recursive fk_solver_; /**< IK  velocity solver */
				KDL::ChainJntToJacSolver jac_solver_; /**< IK  velocity solver */
				// limits
				Eigen::VectorXd q_center_, q_scale_;
				Eigen::VectorXd z_opt_, weights_;
				// computation buffer
				Eigen::JacobiSVD<decltype(KDL::Jacobian::data)> J_svd_;
				Eigen::VectorXd dq_dz_, dC_dz_;
				KDL::Jacobian J_;

			public:

				IKControlSolver(const KDL::Chain& chain_arg, const Eigen::VectorXd& q_min, const Eigen::VectorXd& q_max, const Eigen::VectorXd& q_opt, const Eigen::VectorXd& _weights, double jac_svd_threshold);
				IKControlSolver& operator=(const IKControlSolver&) = delete;
				bool step(KDL::JntArrayVel& state, const KDL::Frame& traget_frame, const KDL::Twist& target_twist, double T, double Kp_rot, double Kp_pos, double Kp_null, double q_reduction_factor, double max_rot_vel, double max_pos_vel, sweetie_bot::logger::Logger& log);
				bool solveFK(const KDL::JntArray& q, KDL::Frame& pose) {
					return fk_solver_.JntToCart(q, pose) >= 0;
				}
				void setSvdThreshold(double threshold) { 
					J_svd_.setThreshold(threshold); 
				}
		};

		struct KinematicChainData 
		{
			// chain info
			std::string name; /**< Kinematic chain name */
			std::vector<std::string> joint_names; /**< Names of joint. */
			std::vector<int> joint_induces; /**< Induces of joints in chain */
			int size; /**< Kinematic chain length. */
			int size_real; /**< Kinematic chain length without fictive joints */
			// chain solver
			std::unique_ptr<IKControlSolver> solver; /**< IK solver */
			KDL::JntArrayVel state; /**< Solver state. */
			KDL::JntArrayVel state_filt; /**< Solver state. */
			// tolerance
			Eigen::VectorXd tolerance;
		};

	protected:

		// COMPONENT INTERFACE 
		// ports
		RTT::InputPort<sensor_msgs::JointState> in_joints_fullpose_port_;
		RTT::OutputPort<sensor_msgs::JointState> out_joints_port_;
		RTT::InputPort<sweetie_bot_kinematics_msgs::RigidBodyState> in_limbs_port_;
		// properties
		double period_;
		std::vector<std::string> chain_names_;
		double n_steps_;
		double alpha_;
		double jac_svd_threshold_;
		double q_reduction_factor_;
		double kp_rot_;
		double kp_pos_;
		double kp_null_;
		double max_rot_vel_, max_pos_vel_, max_jnt_vel_;
		bool ignore_ref_twist_;
		// subservices
		RobotModel * robot_model_;

		// COMPONENT STATE
		// buffers
		sensor_msgs::JointState joints_;
		sensor_msgs::JointState joints_fullpose_;
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
		// helper functions
		Eigen::VectorXd getOrAddVectorProperty(const std::string& name, const Eigen::VectorXd& default_value, const std::string& desc = "");
		bool poseToJointState_impl(const sweetie_bot_kinematics_msgs::RigidBodyState& in, sensor_msgs::JointState& out);

		// operations
		int poseToJointState(const sweetie_bot_kinematics_msgs::RigidBodyState& in, sensor_msgs::JointState& out);
		bool poseToJointStatePublish(const sweetie_bot_kinematics_msgs::RigidBodyState& in, int approx_mode);

	public:

		KinematicsInvControl(const std::string& name);

		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
};

} // namespace motion
} // namespace sweetie_bot

#endif  /*KINEMATICS_INV_CONTROL-COMPONENT_HPP*/

