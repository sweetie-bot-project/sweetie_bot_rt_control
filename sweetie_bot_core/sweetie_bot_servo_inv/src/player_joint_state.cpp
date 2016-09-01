#include <fstream>
#include <sstream>

#include <rtt/Component.hpp>

#include "player_joint_state.hpp"

using namespace RTT;

PlayerJointState::PlayerJointState(std::string const& name) : 
	TaskContext(name)
{
	this->addPort(joints_port)
		.doc("Trajectory read from file.");

	this->addProperty("file_name", file_name)
		.doc("Trajectory file name. Trajectory is stored one sample per line in format: pos1 vel1 [effort1] pos2 vel2 [effort2] ... Lines beginning from # are skipped.");
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

bool PlayerJointState::startHook()
{
	Logger::In in("PlayerJointState");

	std::ifstream file;
	file.exceptions(std::istream::failbit | std::istream::badbit);

	try {
		file.open(file_name.c_str());

		std::string line;
		std::istringstream ss;
		ss.exceptions(std::istream::failbit | std::istream::badbit | std::istream::eofbit);

		sample_index = 0;
		while (getline(file, line)) {
			float value;
				
			ss.str(line);
			sample_index++;

			for(unsigned int i = 0; i < joint_names.size(); i++) {
				ss >> value; 
				trajectory_pos.push_back(value);
				ss >> value;
				trajectory_vel.push_back(value);

				if (effort_presents) {
					ss >> value;
					trajectory_effort.push_back(value);
				}
			}
		}
	} 
	catch (std::ios_base::failure e) {
		log(Error) << "Near line " << sample_index << " error: " << e.what() << endlog();
		return false;
	}

	sample_index = 0;
	sample_dim = joint_names.size();

	joints.name = joint_names;
	joints.position.resize(sample_dim);
	joints.velocity.resize(sample_dim);
	if (effort_presents) joints.effort.resize(sample_dim);
	else joints.effort.resize(0);

	joints_port.setDataSample(joints);

	log(Info) << "PlayerJointState started!" << endlog();
	return true;
}

void PlayerJointState::updateHook()
{
	if (sample_index*sample_dim >= trajectory_pos.size()) {
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



void PlayerJointState::stopHook() {
	Logger::In in("PlayerJointState");
	log(Info) << "PlayerJointState stopped!" << endlog();
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
ORO_CREATE_COMPONENT(PlayerJointState)
