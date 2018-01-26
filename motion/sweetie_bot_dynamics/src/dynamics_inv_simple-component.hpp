#ifndef OROCOS_SWEETIE_BOT_DYNAMICS_INV_SIMPLE_COMPONENT_HPP
#define OROCOS_SWEETIE_BOT_DYNAMICS_INV_SIMPLE_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/os/Timer.hpp>

#include <rbdl/rbdl.h>

#include <geometry_msgs/typekit/WrenchStamped.h>
#include <sensor_msgs/typekit/JointState.h>

#include <sweetie_bot_logger/logger.hpp>
#include <sweetie_bot_robot_model/robot_model.hpp>

#include <sweetie_bot_kinematics_msgs/typekit/RigidBodyState.h>
#include <sweetie_bot_kinematics_msgs/typekit/SupportState.h>
#include <sweetie_bot_kinematics_msgs/typekit/JointStateAccel.h>


namespace sweetie_bot {
namespace motion {

class DynamicsInvSimple : public RTT::TaskContext
{
	protected:

	protected:
		struct ContactState {
			std::string limb_name;
			std::string contact_name;
			int index;
			int body_id;
			std::vector<KDL::Vector> contact_points;
			bool is_active;
		};
	protected: 
		// COMPONENT INTERFACE 
		// ports
		RTT::InputPort<sensor_msgs::JointState> in_joints_port;
		RTT::InputPort<sweetie_bot_kinematics_msgs::RigidBodyState> in_base_port;
		RTT::InputPort<sweetie_bot_kinematics_msgs::SupportState> in_supports_port;
		RTT::InputPort<RTT::os::Timer::TimerId> sync_port;

		RTT::OutputPort<sweetie_bot_kinematics_msgs::JointStateAccel> out_joints_accel_port;
		//RTT::OutputPort<sweetie_bot_kinematics_msgs::SupportState> out_supports_port;
		RTT::OutputPort<sweetie_bot_kinematics_msgs::RigidBodyState> out_wrenches_port;
		// properties
		std::string robot_description;
		std::vector<std::string> legs;
		double tolerance;
		double period;
		// subservices
		RobotModel * robot_model;

		// COMPONENT STATE
	
		// rdbl robot model
        RigidBodyDynamics::Model rbdl_model; 
		// state
		VectorN_t Q;          // pose
		VectorN_t QDot;       // generalized speed
		VectorN_t QDDot;      // generalized accel
		VectorN_t tau;        // generalized forces

		// contact model
		std::vector<ContactState> contacts; // contact list
		MatrixN_t Jc_reserved; // contact jacobian buffer
		VectorN_t JcDotQDot_reserved; // contact jacobian buffer
		VectorN_t lambda_reserved;     // Lagrangian multiplers

		// joint index: fullpose induces of rbdl joints
		std::vector<int> joint_index;
		int n_fullpose_joints;
		
		// buffers
		sensor_msgs::JointState joints;
		sweetie_bot_kinematics_msgs::JointStateAccel joints_accel;
		sweetie_bot_kinematics_msgs::RigidBodyState base;
		sweetie_bot_kinematics_msgs::SupportState supports;
		sweetie_bot_kinematics_msgs::RigidBodyState wrenches;

		// logging
#ifdef SWEETIEBOT_LOGGER
		sweetie_bot::logger::SWEETIEBOT_LOGGER log;
#else
		sweetie_bot::logger::LoggerRTT log;
#endif

	protected:
		bool checkPorts();
		void updateStateFromPortsBuffers();
		void inverseDynamic();
		void publishStateToPorts();

	public:
		DynamicsInvSimple(std::string const& name);

		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
};

} // namespace motion
} // namespace sweetie_bot

#endif /* OROCOS_SWEETIE_BOT_DYNAMICS_INV_SIMPLE_COMPONENT_HPP */
