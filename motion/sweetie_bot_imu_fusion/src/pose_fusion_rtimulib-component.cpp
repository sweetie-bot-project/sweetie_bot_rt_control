#include "pose_fusion_rtimulib-component.hpp"

#include <cstdio>

#include <kdl/frames.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <rtt/Component.hpp>

using namespace RTT;

namespace sweetie_bot {
namespace motion {

PoseFusionRTIMULib::PoseFusionRTIMULib(std::string const& name) :
	RTT::TaskContext(name, PreOperational),
	log(logger::categoryFromComponentName(name))
{
	if (!log.ready()) {
		RTT::Logger::In in("PoseFusionRTIMULib");
		RTT::log(RTT::Error) << "Logger is not ready!" << RTT::endlog();
	}

	// PORTS
	this->addPort("out_imu", imu_port)
		.doc("IMU state.");
	this->addPort("out_base", base_port)
		.doc("Base orientation and speed estimate.");
	this->addPort("out_tf", tf_port)
		.doc("Base orientation for tf.");

	// PROPERTIES
	this->addProperty("rtimulib_config_file", rtimulib_config_file)
		.doc("RTIMULib configuration file name (without .ini extension).")
		.set("RTIMULib");
	this->addProperty("rtimulib_config_path", rtimulib_config_path)
		.doc("RTIMULib configuration file directory.");
	this->addProperty("imu_frame", imu_frame)
		.doc("IMU frame name.")
		.set("base_link");
	this->addProperty("odometry_frame", odometry_frame)
		.doc("Inertial frame name.")
		.set("odom_combined");
	this->addProperty("tf_prefix", tf_prefix)
		.doc("tf prefix for published transform.")
		.set("real");
	this->addProperty("compass_enable", compass_enable)
		.doc("Use magnetometer during data fussion.")
		.set(true);

	log(INFO) << "PoseFusionRTIMULib constructed !" << endlog();
}

bool PoseFusionRTIMULib::configureHook()
{
	// initialize IMU
	
	// check if configuration file exists
	std::string filename = rtimulib_config_path + "/" + rtimulib_config_file + ".ini";
	FILE * file = fopen(filename.c_str(), "r");
	if ( ! file ) {
		// file does not exists or unreadible	
		log(ERROR) << "Unable to open configuration file '" << filename << "': " << strerror(errno) << RTT::endlog();
		return false;
	}
	fclose(file);

	// load settings	
	settings = std::make_shared<RTIMUSettings>(rtimulib_config_path.c_str(), rtimulib_config_file.c_str());
	if (! settings->loadSettings() ) {
		log(ERROR) << "Unable to load RTIMULib configuration." << endlog();
		return false;
	}

	// Create IMU object.
    imu.reset( RTIMU::createIMU(settings.get()) );

    // Initialise the imu object
    if ((imu == nullptr) || (imu->IMUType() == RTIMU_TYPE_NULL) || !imu->IMUInit())
    {
		log(ERROR) << "Unable to init IMU." << endlog();
		return false;
    }

    // Set the Fusion coefficient
    imu->setSlerpPower(0.02);
    // Enable the sensors
    imu->setGyroEnable(true);
    imu->setAccelEnable(true);
    imu->setCompassEnable(compass_enable);

	// Set recommended period
	this->setPeriod(imu->IMUGetPollInterval() / 1000.0);

	// Buffers
	imu_msg.header.frame_id = imu_frame;
	base.name.resize(1);
	base.frame.resize(1);
	base.twist.clear();
	base.name[0] = "base_link";

	base_tf.transforms.resize(1);
	base_tf.transforms[0].header.frame_id = odometry_frame;
	if (tf_prefix != "") {
		base_tf.transforms[0].child_frame_id = tf_prefix + "/base_link";
	}
	else {
		base_tf.transforms[0].child_frame_id = "base_link";
	}


	log(INFO) << "PoseFusionRTIMULib is configured !" << endlog();
	return true;
}

bool PoseFusionRTIMULib::startHook()
{
    imu->resetFusion();

	log(INFO) << "PoseFusionRTIMULib is started !" << endlog();
	return true;
}

void PoseFusionRTIMULib::updateHook()
{
	const double G_TO_MPSS = 9.80665;

	if (imu->IMURead())
	{
		ros::Time stamp = ros::Time::now();
		RTIMU_DATA imu_data = imu->getIMUData();

		// IMU message
		imu_msg.header.stamp = stamp;

		imu_msg.orientation.x = imu_data.fusionQPose.x(); 
		imu_msg.orientation.y = imu_data.fusionQPose.y(); 
		imu_msg.orientation.z = imu_data.fusionQPose.z(); 
		imu_msg.orientation.w = imu_data.fusionQPose.scalar(); 

		imu_msg.angular_velocity.x = imu_data.gyro.x();
		imu_msg.angular_velocity.y = imu_data.gyro.y();
		imu_msg.angular_velocity.z = imu_data.gyro.z();

		imu_msg.linear_acceleration.x = imu_data.accel.x() * G_TO_MPSS;
		imu_msg.linear_acceleration.y = imu_data.accel.y() * G_TO_MPSS;
		imu_msg.linear_acceleration.z = imu_data.accel.z() * G_TO_MPSS;

		imu_port.write(imu_msg);

		// base link 
		base.header.stamp = stamp;
		base.frame[0].M = KDL::Rotation::Quaternion(imu_data.fusionQPose.x(), imu_data.fusionQPose.y(), imu_data.fusionQPose.z(), imu_data.fusionQPose.scalar());
		base.frame[0].p = KDL::Vector(imu_data.fusionPose.x(), imu_data.fusionPose.y(), imu_data.fusionPose.z());
		base_port.write(base);

		// tf
		base_tf.transforms[0].header.stamp = stamp;
		tf::transformKDLToMsg(base.frame[0], base_tf.transforms[0].transform);

		tf_port.write(base_tf);
	}
}

void PoseFusionRTIMULib::stopHook() 
{

	log(INFO) << "PoseFusionRTIMULib is stopped !" << endlog();
}

void PoseFusionRTIMULib::cleanupHook() 
{
	settings.reset();
	imu.reset();

	log(INFO) << "PoseFusionRTIMULib is cleaned up !" << endlog();
}

}
}


/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(PoseFusionRTIMULib)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(sweetie_bot::motion::PoseFusionRTIMULib)
