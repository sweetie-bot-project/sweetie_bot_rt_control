#include "pose_fusion_rtimulib-component.hpp"

#include <cstdio>

#include <kdl/frames.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <rtt/Component.hpp>

#include <sweetie_bot_orocos_misc/message_checks.hpp>

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
	this->addPort("in_base_ref", base_ref_port)
		.doc("Robot base reference pose (RTIMULib does not produce position estimate).");
	this->addPort("sync_step", sync_port)
		.doc("Timer event indicating beginig of next control cycle."); 

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
	this->addProperty("pose_publish_divider", pose_publish_divider)
		.doc("Publish pose estimate only each pose_publish_divider cycle.")
		.set(10);
	this->addProperty("filter_startup_time", filter_startup_time)
		.doc("During this period (seconds) IMU filter output is considered not valid.")
		.set(1.0);
	this->addProperty("period", period)
		.doc("Control cycle duration (s). ")
		.set(1.0);

	log(INFO) << "PoseFusionRTIMULib constructed !" << endlog();
}

bool PoseFusionRTIMULib::configureHook()
{
	// initialize IMU
	if (pose_publish_divider <= 0) {
		log(ERROR) << "Publish divider must be positive." << endlog();
		return false;
	}
	
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
	// this->setPeriod(imu->IMUGetPollInterval() / 1000.0);

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

	// set data samples
	base_port.setDataSample(base);
	tf_port.setDataSample(base_tf);
	imu_port.setDataSample(imu_msg);

	log(INFO) << "PoseFusionRTIMULib is configured !" << endlog();
	return true;
}

bool PoseFusionRTIMULib::startHook()
{
	imu->resetFusion();
	base.frame[0] = KDL::Frame::Identity();
	pose_publish_cycle = 0;
	startup = true;

	// get data samples
	base_ref_port.getDataSample(base_ref);
	base_ref_port.getDataSample(prev_base_ref);

	RTT::os::Timer::TimerId timer_id;
	sync_port.readNewest(timer_id);

	log(INFO) << "PoseFusionRTIMULib is started !" << endlog();
	return true;
}

void PoseFusionRTIMULib::updateHook()
{
	const double G_TO_MPSS = 9.80665;

	ros::Time stamp = ros::Time::now();
	RTIMU_DATA imu_data;
	if (imu->IMURead())
	{
		imu_data = imu->getIMUData();

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
	}

	if (startup && pose_publish_cycle*getPeriod() > filter_startup_time) {
		// check if reference pose is valid
		if (base_ref_port.read(base_ref, true) != RTT::NoData && isValidRigidBodyStateNameFrame(base_ref)) {
			// store current postion 
			base = base_ref;
			// calculate rotation correction for IMU pose
			KDL::Rotation R_imu = KDL::Rotation::Quaternion(imu_data.fusionQPose.x(), imu_data.fusionQPose.y(), imu_data.fusionQPose.z(), imu_data.fusionQPose.scalar());
			R_corr = R_imu.Inverse() * base_ref.frame[0].M;
			// init position shift variablares
			prev_base_ref = base_ref;
			pos_shift_ref = KDL::Vector::Zero();
			velocity_ref = KDL::Vector::Zero();
			prev_base_ref_good = false;
			// startup is finished
			startup = false;
		}
	} 
	else if (!startup) {
		// check data ports
		RTT::os::Timer::TimerId timer_id;
		if (sync_port.read(timer_id) == RTT::NewData) {
			if (base_ref_port.readNewest(base_ref, false) != RTT::NewData && isValidRigidBodyStateNameFrame(base_ref)) {
				// calculate shift
				if (prev_base_ref_good) {
					pos_shift_ref = prev_base_ref.frame[0].M.Inverse( base_ref.frame[0].p - prev_base_ref.frame[0].p );
					velocity_ref = prev_base_ref.twist[0].vel;
				}
				else {
					pos_shift_ref = KDL::Vector::Zero();
					velocity_ref = KDL::Vector::Zero();
				}
				// save previous pose
				prev_base_ref = base_ref;
				prev_base_ref_good = true;
			}
			else {
				pos_shift_ref = KDL::Vector::Zero();
				velocity_ref = KDL::Vector::Zero();
				prev_base_ref_good = false;
			}
		}

		// base link
		base.header.stamp = stamp;
		// get orientation from IMU
		base.frame[0].M = KDL::Rotation::Quaternion(imu_data.fusionQPose.x(), imu_data.fusionQPose.y(), imu_data.fusionQPose.z(), imu_data.fusionQPose.scalar()) * R_corr;
		// integrate position
		base.frame[0].p += base.frame[0].M * (getPeriod()/period * pos_shift_ref);

		// trottle pose publishing
		if (pose_publish_cycle % pose_publish_divider == 0) {
			// publish results
			base_port.write(base);
			// tf
			base_tf.transforms[0].header.stamp = stamp;
			tf::transformKDLToMsg(base.frame[0], base_tf.transforms[0].transform);
			tf_port.write(base_tf);
		}
	}

	// increase cycle counter
	pose_publish_cycle++;
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
