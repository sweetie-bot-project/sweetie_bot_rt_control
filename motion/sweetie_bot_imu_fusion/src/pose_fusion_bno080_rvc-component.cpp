#include "pose_fusion_bno080_rvc-component.hpp"

#include <cstdio>
#include <cstring>
#include <numeric>

extern "C" {
#include <termios.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
}

#include <kdl/frames.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <rtt/Component.hpp>
#include <rtt/extras/FileDescriptorActivity.hpp>

#include <sweetie_bot_orocos_misc/message_checks.hpp>

using namespace RTT;

inline std::ostream& operator<<(std::ostream& s, const KDL::Vector& v) 
{
	s << "[" << v.x() << " " << v.y() << " " << v.z() << " ]";
	return s;
}

namespace sweetie_bot {
namespace motion {

//Convinence macro fo logging.
std::ostream& resetfmt(std::ostream& s) {
    s.copyfmt(std::ios(nullptr));
    return s;
}

PoseFusionBNO080RVC::PoseFusionBNO080RVC(std::string const& name) :
	RTT::TaskContext(name, PreOperational),
	log(logger::categoryFromComponentName(name))
{
	if (!log.ready()) {
		RTT::Logger::In in("PoseFusionBNO080RVC");
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
		.doc("Robot base reference pose (BNO080RVC does not produce position estimate).");
	this->addEventPort("sync", sync_port)
		.doc("Timer event indicating beginig of next control cycle."); 

	// PROPERTIES
    this->addProperty("port_name", port_name)
            .doc("Serial port device name.");
    this->addProperty("baudrate", baudrate)
            .doc("Serial port baudrate.")
            .set(115200);
	this->addProperty("imu_frame", imu_frame)
		.doc("IMU frame name.")
		.set("base_link");
	this->addProperty("odometry_frame", odometry_frame)
		.doc("Inertial frame name.")
		.set("odom_combined");
	this->addProperty("tf_prefix", tf_prefix)
		.doc("tf prefix for published transform.")
		.set("real");
	this->addProperty("filter_startup_time", filter_startup_time)
		.doc("During this period (seconds) IMU filter output is considered not valid.")
		.set(1.0);
	this->addProperty("period", period)
		.doc("Control cycle duration (s). ")
		.set(1.0);

    this->setActivity(new extras::FileDescriptorActivity(60, nullptr, "IMURVCPortActivity"));

	log(INFO) << "PoseFusionBNO080RVC constructed !" << endlog();
}

int PoseFusionBNO080RVC::openSerialPort(const std::string& port_name, int baudrate) 
{
    struct termios tty;
	int port_fd;

    port_fd = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK );
    if (port_fd == -1) {
        log(ERROR) << "open() serial port \"" << port_name << "\" failed: " << strerror(errno) << endlog();
        return -1;
    }
    // configure serial port
    if (tcgetattr (port_fd, &tty) != 0) {
        log(ERROR) << "tcgetattr() failed: " << strerror(errno) << endlog();
        return -1;
    }

    // 8-bits, 1 STOP bit, enable receiver, ignore modem lines
    //tty.c_cflag = CS8 | CREAD | CSTOPB | CLOCAL;
    tty.c_cflag = CS8 | CREAD | CLOCAL;
    // no signaling chars, no echo, no canonical processing
    tty.c_lflag = 0;
    // no special input processing
    tty.c_iflag = 0;
    // no special output processing
    tty.c_oflag = 0;
    // set speed
    int ret;
    switch (baudrate) {
        case 9600:
            ret = cfsetspeed (&tty, B9600);
            break;
        case 19200:
            ret = cfsetspeed (&tty, B19200);
            break;
        case 38400:
            ret = cfsetspeed (&tty, B38400);
            break;
        case 57600:
            ret = cfsetspeed (&tty, B57600);
            break;
        case 115200:
            ret = cfsetspeed (&tty, B115200);
            break;
        case 230400:
            ret = cfsetspeed (&tty, B230400);
            break;
        default:
            log(ERROR) << "Incorrect baudrate property value: " << baudrate << endlog();
            return -1;
    }
    if (ret) {
        log(ERROR) << "cfsetspeed() failed: " << strerror(errno) << endlog();
        return -1;
    }
    // special properties
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout, so read will not block forever
    // configure port
    if (tcsetattr (port_fd, TCSANOW, &tty) != 0) {
        log(ERROR) << "tcsetattr() failed: " << strerror(errno) << endlog();
        return -1;
    }
    // set low_latency flag
    struct serial_struct serial;
    if (ioctl(port_fd, TIOCGSERIAL, &serial) == -1) {
        log(WARN) << "Unable to get serial_struct. ioctl() failed: " << strerror(errno) << endlog();
    }
    else {
        serial.flags |= ASYNC_LOW_LATENCY;
        if (ioctl(port_fd, TIOCSSERIAL, &serial) == -1) {
            log(WARN) << "Unable to set low_latency flag. ioctl() failed: " << strerror(errno) << endlog();
        }
    }
	return port_fd;
}

bool PoseFusionBNO080RVC::configureHook()
{
    // setup FileDescriptorActivity
    extras::FileDescriptorActivity * activity = dynamic_cast<extras::FileDescriptorActivity *>(this->getActivity());
    if (! activity) {
        log(ERROR) << "Incompatible activity type."  << endlog();
        return false;
    }

	// open serial port and associate it with activity
	this->port_fd = openSerialPort(this->port_name, this->baudrate);
    if (port_fd == -1) {
        log(ERROR) << "Unable to open and configure " << this->port_name << endlog();
        return false;
    }
    activity->watch(port_fd);

	// Buffers
	imu_msg.header.frame_id = imu_frame;
	base.name.resize(1);
	base.frame.resize(1);
	base.twist.resize(1);
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

	log(INFO) << "PoseFusionBNO080RVC is configured !" << endlog();
	return true;
}

bool PoseFusionBNO080RVC::startHook()
{
	// reset packet parser
	recv_state = HEADER1;
	buffer_index = 0;
	buffer_bytes_recieved = 0;
	packet_index = 0;

	// reset fusion algorithm
	base.frame[0] = KDL::Frame::Identity();
	startup = true;
	imu_ready = false;
	base_yaw_correction = 0.0;

	// get data samples
	base_ref_port.getDataSample(base_ref);
	base_ref_port.getDataSample(base_ref_prev);

	RTT::os::Timer::TimerId timer_id;
	sync_port.readNewest(timer_id);

	log(INFO) << "PoseFusionBNO080RVC is started !" << endlog();
	return true;
}


int PoseFusionBNO080RVC::parseIMUStream()
{
	// read new data on port
	ssize_t retval = TEMP_FAILURE_RETRY(read(port_fd, buffer.raw.buffer + buffer_bytes_recieved, sizeof(buffer.raw.buffer) - buffer_bytes_recieved));
	if (retval == 0) return retval;
	else if (retval == -1) {
		log(ERROR) << "Read serial port failed:" << strerror(errno) << endlog();
		this->exception();
		return retval;
	}

	buffer_bytes_recieved += ulong(retval);

	if (log(DEBUG)) {
		log() << "READ buffer (" << buffer_bytes_recieved << " bytes):" << std::hex << std::setw(2) << std::setfill('0');
		for (ulong i = 0; i < buffer_bytes_recieved; i++) log() << uint32_t( buffer.raw.buffer[i] ) << " ";
		log() << resetfmt << std::endl;
		log() << "recv_state = " << recv_state << ", recv_index = " << buffer_index << endlog();
	}

	while(buffer_index < buffer_bytes_recieved) {
		unsigned char * header_ptr;
		const double G_TO_MPSS = 9.80665;

		log(DEBUG) << "recv_state = " << recv_state << ", recv_index = " << buffer_index << endlog();
		switch (recv_state) {
			case HEADER1:
				header_ptr = (unsigned char *) memchr(buffer.raw.buffer + buffer_index, 0xAA, buffer_bytes_recieved - buffer_index);
				if (header_ptr == nullptr) {
					// nothing found: dump buffer
					buffer_index = 0; 
					buffer_bytes_recieved = 0;
					// wait new data in HEADER1 state
					break;
				}
				// calculate header postiton
				buffer_index = header_ptr - buffer.raw.buffer;
				// check if next symbol is available
				if (buffer_index + 1 >= buffer_bytes_recieved) {
					// next symbol is not read yet: dump buffer except for the first symbol, align message
					buffer_index = 1;
					buffer_bytes_recieved = 1;
					buffer.raw.buffer[0] = 0xAA;
					// wait new data in HEADER2 state
					recv_state = HEADER2;
					break;
				}
				// next state
				recv_state = HEADER2;
				buffer_index += 1;

			case HEADER2:
				// check second symbol
				if (buffer.raw.buffer[buffer_index] != 0xAA) {
					// header is not found: found 0xAA and second symbol is no 0xAA
					// skip header
					recv_state = HEADER1;
					buffer_index += 1;
					// wait new data in HEADER state
					break;
				}
				// header found: align message
				// move buffer content to aling it with fields structure
				if (buffer_index != 1) {
					memmove(&buffer.raw.buffer, buffer.raw.buffer + (buffer_index-1), buffer_bytes_recieved - buffer_index);
					buffer_bytes_recieved -= (buffer_index-1);
				}
				// skip header
				buffer_index = 2;
				// next state
				imu_msg.header.stamp = ros::Time::now();
				recv_state = DATA;
				break;

			case DATA:
				// buffer content is properly aligned, buffer_index points on data start
			
				// check that we have full frame
				if (buffer_bytes_recieved < PACKET_SIZE) {
					// frame is not full: wait for next read
					// skip all data
					buffer_index = buffer_bytes_recieved;
					break;
				}

				// check checksum
				if (buffer.fields.crc != std::accumulate(&buffer.fields.index, &buffer.fields.crc, uint8_t(0))) {
					log(ERROR) << "CRC did not match!" << endlog();
					// skip header and try to parse next frame
					recv_state = HEADER1;
					buffer_index = 1;
					break;
				}
				// check reserved bytes
				if ( (buffer.fields.reserved1 | buffer.fields.reserved2 | buffer.fields.reserved3) != 0)
				{
					log(ERROR) << "Reserved bytes are not zeros! " << endlog();
					// skip header and try to parse next frame
					recv_state = HEADER1;
					buffer_index = 1;
					break;
				}
				// check index
				if (imu_ready && (uint8_t(packet_index+1) != buffer.fields.index))
				{
					log(WARN) << "Packet index do not match! Expected " << uint8_t(packet_index+1) << ", received " << buffer.fields.index << endlog();
				}

				// extract package content
		
				// index
				packet_index = buffer.fields.index;
				// orientation
				double pitch = (buffer.fields.pitch.num / 100.0) / (180.0 / M_PI);
				double roll = (buffer.fields.roll.num / 100.0) / (180.0 / M_PI);
				// apply yaw correction
				base_yaw = (buffer.fields.yaw.num / 100.0) / (180.0 / M_PI) + base_yaw_correction;
				// convert Euler to Quaternion
				tf2::Quaternion quat;
				quat.setRPY(roll, pitch, base_yaw);
				tf2::convert(quat, imu_msg.orientation);
				// linear acceleration
				imu_msg.linear_acceleration.x = (buffer.fields.accelx.num / 1000.0) * G_TO_MPSS;
				imu_msg.linear_acceleration.y = (buffer.fields.accely.num / 1000.0) * G_TO_MPSS;
				imu_msg.linear_acceleration.z = (buffer.fields.accelz.num / 1000.0) * G_TO_MPSS;
				// assume IMU is ready
				imu_ready = true;

				if (log(DEBUG)) {
					log() << "IMU message " << imu_msg << endlog();
				}

				// publish imu
				imu_port.write(imu_msg);
				
				// skip parsed data 
				buffer_index = PACKET_SIZE;
				recv_state = HEADER1;
				break;
		}
	}
	return retval;
}

void PoseFusionBNO080RVC::updateHook()
{
	log() << "updateHook()" << endlog();

	// get activity
    extras::FileDescriptorActivity * activity = dynamic_cast<extras::FileDescriptorActivity *>(this->getActivity());
    if (! activity) {
        this->exception();
        log(ERROR) << "Incompatible activity type."  << endlog();
		return;
    }
    if (activity->hasError()) {
        log(ERROR) << "FileDescriptorActivity error."  << endlog();
        this->exception();
        return;
    }

	// get current timestamp
	ros::Time timestamp_sync = ros::Time::now();

	// process imu output
    if (activity->isUpdated(port_fd)) {
		// parse input stream until it is exhausted
		while (parseIMUStream() == 0);
	}

	// check sync port
	RTT::os::Timer::TimerId timer_id;
	if (sync_port.read(timer_id) == RTT::NewData) {
		if (startup && imu_ready) {
			// perform state intialization: 

			// check if reference pose is valid
			if (base_ref_port.readNewest(base_ref, true) != RTT::NoData && isValidRigidBodyStateNameFrame(base_ref)) {
				// store current postion 
				base = base_ref;
				// calculate rotation correction for IMU pose
				double roll, pitch, yaw;
				base_ref.frame[0].M.GetRPY(roll, pitch, yaw);
				base_yaw_correction += yaw - base_yaw;
				// init position shift variablares
				base_ref_p_shift = KDL::Vector::Zero();
				base_ref_prev = base_ref;
				base_yaw = yaw;
				// startup is finished
				startup = false;
			}
		} 
		else {
			// use orientation from IMU	
			base.frame[0].M = KDL::Rotation::Quaternion(imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w);
			// use expected XY-shift in path frame to calculate estimated base position change
			base.frame[0].p += KDL::Rotation::RotZ(base_yaw) * base_ref_p_shift;
			// use z coordinate from odometry
			base.frame[0].p.z(base_ref.frame[0].p.z());
			// use twist  odometry
			base.twist[0] = base_ref.twist[0];

			// publish pose
			base.header.stamp = timestamp_sync;
			base_port.write(base);

			// publish tf
			base_tf.transforms[0].header.stamp = timestamp_sync;
			tf::transformKDLToMsg(base.frame[0], base_tf.transforms[0].transform);
			tf_port.write(base_tf);
		}

		//
		// get next reference pose: reference pose is applied on next control cycle
		//
		// store previos pose
		base_ref_prev = base_ref;
		// get next pose
		if (base_ref_port.readNewest(base_ref, true) != RTT::NoData && isValidRigidBodyStateNameFrame(base_ref)) {
			double roll, pitch, yaw;
			base_ref_prev.frame[0].M.GetRPY(roll, pitch, yaw);
			// shift in path coordinate system
			base_ref_p_shift = KDL::Rotation::RotZ(-yaw)*(base_ref.frame[0].p - base_ref_prev.frame[0].p);
		}
		else {
			log(WARN) << "Missing or invalid base_ref sample." << endlog();
			// assume shift equal to zero
			base_ref_p_shift = KDL::Vector::Zero();
		}


		if (log(DEBUG)) {
			log() << "IMU fusion: base_yaw = " << base_yaw << ", base_yaw_correction = " << base_yaw_correction << ", base_ref_p_shift = " << base_ref_p_shift << endlog();
		}
	}

}

void PoseFusionBNO080RVC::stopHook() 
{

	log(INFO) << "PoseFusionBNO080RVC is stopped !" << endlog();
}

void PoseFusionBNO080RVC::cleanupHook() 
{
    extras::FileDescriptorActivity * activity = dynamic_cast<extras::FileDescriptorActivity *>(this->getActivity());
    if (! activity) {
        log(ERROR) << "Incompatible activity type."  << endlog();
		return;
    }
    activity->unwatch(port_fd);

    if (TEMP_FAILURE_RETRY(close(port_fd))) {
        log(ERROR) << "close() serial port failed: " << strerror(errno) << endlog();
    }
	log(INFO) << "PoseFusionBNO080RVC is cleaned up !" << endlog();
}

}
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(PoseFusionBNO080RVC)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(sweetie_bot::motion::PoseFusionBNO080RVC)
