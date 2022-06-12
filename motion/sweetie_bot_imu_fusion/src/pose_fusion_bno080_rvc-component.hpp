#ifndef OROCOS_SWEETIE_BOT_POSE_FUSION_BNO080_RVC_COMPONENT_HPP
#define OROCOS_SWEETIE_BOT_POSE_FUSION_BNO080_RVC_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/os/Timer.hpp>

#include <sensor_msgs/typekit/Imu.h>
#include <tf2_msgs/typekit/TFMessage.h>

#include <sweetie_bot_logger/logger.hpp>

#include <sweetie_bot_kinematics_msgs/typekit/RigidBodyState.h>


namespace sweetie_bot {
namespace motion {

class PoseFusionBNO080RVC : public RTT::TaskContext
{
	protected:
		static const int PACKET_SIZE = 19;
		static const int BUFFER_SIZE = 40;

	protected:
		union Register {
			std::int16_t num;
			struct __attribute__ ((packed)) Bytes
			{
				// The order of these bytes matters
				std::uint8_t msb;
				std::uint8_t lsb;
			} bytest;
		};

		struct __attribute__ ((packed)) Packet {
			std::uint8_t padding;
			std::uint8_t header1;
			std::uint8_t header2;
			std::uint8_t index;
			Register yaw;
			Register pitch;
			Register roll;
			Register accelx;
			Register accely;
			Register accelz;
			std::uint8_t reserved1;
			std::uint8_t reserved2;
			std::uint8_t reserved3;
			std::uint8_t crc;
		};
		union __attribute__ ((packed)) PacketBuffer {
			Packet fields;
			struct Raw {
				std::uint8_t padding;
				std::uint8_t buffer[BUFFER_SIZE];
			} raw;
		};

	protected:
		// Logger
#ifdef SWEETIEBOT_LOGGER
		logger::SWEETIEBOT_LOGGER log;
#else
		logger::LoggerRTT log;
#endif

		// component state
	
		// packet parser state
		enum { HEADER1, HEADER2, DATA } recv_state; // packet parser state
		PacketBuffer buffer; // receive buffer
		int buffer_index;  // packet parser postion
		int buffer_bytes_recieved; // reviced data
		int packet_index;  // number of received packet
		int port_fd; // serial port file descriptor

		// fusion state
		bool startup; // IMU fusion is in startup mode
		bool imu_ready; // IMU data in imu_msg and base_yaw is valid
		KDL::Vector base_ref_p_shift; // reference pose shift during last control cycle in base path frame
		double base_yaw; // estimated base orientataon
		double base_yaw_correction; // angle between odom_combined x-axis and x-axis of absolute IMU frame
		sweetie_bot_kinematics_msgs::RigidBodyState base; // estimated base_link pose

		// buffers
		sensor_msgs::Imu imu_msg; 
		sweetie_bot_kinematics_msgs::RigidBodyState base_ref, base_ref_prev; // reference signal
		tf2_msgs::TFMessage base_tf;

	// COMPONENT INTERFACE
	protected: 
		// PORTS
		RTT::InputPort<sweetie_bot_kinematics_msgs::RigidBodyState> base_ref_port;
		RTT::InputPort<RTT::os::Timer::TimerId> sync_port;
		RTT::OutputPort<sweetie_bot_kinematics_msgs::RigidBodyState> base_port;
		RTT::OutputPort<sensor_msgs::Imu> imu_port;
		RTT::OutputPort<tf2_msgs::TFMessage> tf_port;

		// PROPERTIES
		std::string port_name;
		int baudrate;
		std::string imu_frame;
		std::string odometry_frame;
		std::string tf_prefix;
		double filter_startup_time;
		double period;

	public:
		PoseFusionBNO080RVC(std::string const& name);

		int parseIMUStream();
		int openSerialPort(const std::string& port_name, int baudrate);

		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
};

}
}
#endif
