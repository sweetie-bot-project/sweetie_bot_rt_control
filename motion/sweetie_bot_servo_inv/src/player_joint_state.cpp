#include <fstream>
#include <sstream>
#include <system_error>

#include <rtt/Component.hpp>

#include "player_joint_state.hpp"

using namespace RTT;
using sweetie_bot::logger::Logger;

namespace sweetie_bot 
{

PlayerJointState::PlayerJointState(std::string const& name) : 
	TaskContext(name, PreOperational),
	log(logger::categoryFromComponentName(name))
{
	if (!log.ready()) {
		RTT::Logger::In in("PlayerJointState");
		RTT::log(RTT::Error) << "Logger is not ready!" << RTT::endlog();
		this->fatal();
		return;
	}
	
	this->addPort("out_joints_fixed", joints_port)
		.doc("Trajectory read from file.");
	this->addEventPort("sync", sync_port)
		.doc("Timer sync port.");

	this->addProperty("file_name", file_name)
		.doc("Trajectory file name. Trajectory is stored one sample per line in format: pos1 vel1 [effort1] pos2 vel2 [effort2] ... Lines beginning with '#' are skipped.");
	this->addProperty("joint_names", joint_names)
		.doc("Name of joints.");
	this->addProperty("cycle", cycle)
		.doc("Cycle file content.")
		.set(false);
	this->addProperty("effort_presents", effort_presents)
		.doc("Set true if file contains desired torque (effort).")
		.set(false);
	this->addProperty("skip_first_column", skip_first_column)
		.doc("First column is skipped.")
		.set(false);
}

bool PlayerJointState::configureHook()
{
	sample_dim = joint_names.size(); 

	std::ifstream file(file_name);
	std::istringstream ss;
	std::string line, skip;
	sample_index = 0;
	while (getline(file, line)) {
		double value;
			
		ss.str(line);
		ss.seekg(0);
		sample_index++;

		if (ss.peek() == '#') continue;		

		for(unsigned int i = 0; i < sample_dim; i++) {
			if (skip_first_column) ss >> skip; 
			ss >> value; 
			trajectory_pos.push_back(value);
			ss >> value;
			trajectory_vel.push_back(value);

			if (effort_presents) {
				ss >> value;
				trajectory_effort.push_back(value);
			}
		}

		if (ss.fail())  {
			log(ERROR) << "Unable parse line " << sample_index << "." << endlog();
			return false;
		}
	}
	if (file.fail() && !file.eof()) {
		log(ERROR) << "ERROR reading line " << sample_index << " at " << file_name << ": " << std::error_code(errno, std::system_category()).message() << endlog();
		return false;
	}

	joints.name.resize(sample_dim);
	joints.position.resize(sample_dim);
	joints.velocity.resize(sample_dim);
	if (effort_presents) joints.effort.resize(sample_dim);
	else joints.effort.resize(0);
	joints_port.setDataSample(joints);

	joints.name = joint_names;

	log(INFO) << "PlayerJointState configured!" << endlog();
	return true;
}


bool PlayerJointState::startHook()
{
	sample_index = 0;

	RTT::os::Timer::TimerId timer_id;
	sync_port.readNewest(timer_id);

	log(INFO) << "PlayerJointState started!" << endlog();
	return true;
}


void PlayerJointState::updateHook()
{
	RTT::os::Timer::TimerId timer_id;

	if (this->getPeriod() != 0.0 || sync_port.read(timer_id) == NewData) {
		if ((sample_index + 1)*sample_dim >= trajectory_pos.size()) {
			if (cycle) sample_index = 0;
			else this->stop();
		}

		int begin = sample_dim * sample_index;
		int end = begin + sample_dim;

		std::copy(&trajectory_pos[begin], &trajectory_pos[end], joints.position.begin());
		std::copy(&trajectory_vel[begin], &trajectory_vel[end], joints.velocity.begin());
		if (effort_presents) {
			std::copy(&trajectory_effort[begin], &trajectory_effort[end], joints.effort.begin());
		}

		joints_port.write(joints);
		sample_index++;
	}
}


void PlayerJointState::stopHook() {
	log(INFO) << "PlayerJointState stopped!" << endlog();
}

}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(PlayerJointState)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(sweetie_bot::PlayerJointState)
