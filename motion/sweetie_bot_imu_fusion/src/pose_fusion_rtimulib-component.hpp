#ifndef OROCOS_SWEETIE_BOT_POSE_FUSION_RTIMULIB_COMPONENT_HPP
#define OROCOS_SWEETIE_BOT_POSE_FUSION_RTIMULIB_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/os/Timer.hpp>

#include <rtimulib/RTIMULib.h>

#include <sensor_msgs/typekit/Imu.h>
#include <tf2_msgs/typekit/TFMessage.h>

#include <sweetie_bot_logger/logger.hpp>

#include <sweetie_bot_kinematics_msgs/typekit/RigidBodyState.h>


namespace sweetie_bot {
namespace motion {

class PoseFusionRTIMULib : public RTT::TaskContext
{
	protected:
		// Logger
#ifdef SWEETIEBOT_LOGGER
		logger::SWEETIEBOT_LOGGER log;
#else
		logger::LoggerRTT log;
#endif

		// buffers
		sensor_msgs::Imu imu_msg;
		sweetie_bot_kinematics_msgs::RigidBodyState base;
		sweetie_bot_kinematics_msgs::RigidBodyState base_ref;
		tf2_msgs::TFMessage base_tf;

		// IMU state
		std::shared_ptr<RTIMUSettings> settings;
		std::shared_ptr<RTIMU> imu;

		// component state
		int pose_publish_cycle; // number of execution cycle
		KDL::Rotation R_corr; // difference between IMU orientaton at startup and reference orientaton
		bool R_corr_valid; // difference is caluculated and contains valid value

	// COMPONENT INTERFACE
	protected: 
		// PORTS
		RTT::InputPort<sweetie_bot_kinematics_msgs::RigidBodyState> base_ref_port;
		RTT::OutputPort<sweetie_bot_kinematics_msgs::RigidBodyState> base_port;
		RTT::OutputPort<sensor_msgs::Imu> imu_port;
		RTT::OutputPort<tf2_msgs::TFMessage> tf_port;

		// PROPERTIES
		std::string rtimulib_config_path;
		std::string rtimulib_config_file;
		std::string imu_frame;
		std::string odometry_frame;
		std::string tf_prefix;
		bool compass_enable;
		int pose_publish_divider;
		double filter_startup_time;

	public:
		PoseFusionRTIMULib(std::string const& name);

		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
};

}
}
#endif
