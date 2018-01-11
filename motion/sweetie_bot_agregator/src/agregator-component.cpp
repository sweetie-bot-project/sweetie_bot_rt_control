#include "agregator-component.hpp"

#include <rtt/Component.hpp>
#include <iostream>

#include <sweetie_bot_orocos_misc/joint_state_check.hpp>

using namespace std;
using namespace RTT;

namespace sweetie_bot {
namespace motion {

Agregator::Agregator(string const& name) : TaskContext(name),
	log(logger::categoryFromComponentName(name))
{
  // Ports 
  this->addEventPort("in_joints", input_port_joint_state_)
   .doc( "Messages received on this port is used to update full robot pose buffered by component. Only present joints are updated." );
  this->addPort("out_joints_sorted", output_port_joint_state_)
   .doc( "Port publishes full robot pose buffered by component. It is sorted by kinematics chains." );

  this->addPort("in_support", input_port_support_state_)
   .doc( "Messages received on this port is used to update the contact list. Only present limbs is updated." );
  this->addPort("out_support_sorted", output_port_support_state_ )
   .doc( "Port publishes full contact list. List is publised only after updated." );

  this->addEventPort("sync_step", sync_port_)
   .doc("Timer event indicating the beginning of next control cycle.");
  // Properties
  this->addProperty("publish_on_timer", publish_on_timer_)
    .doc("Publish robot pose when sync port message is received.").set(true);
  this->addProperty("publish_on_event", publish_on_event_)
    .doc("Publish robot pose when partial pose is received.").set(true);
  // opertaions
  this->addOperation("setSupportState", &Agregator::setSupportState, this, OwnThread)
	.doc("Change buffered SupportState. Set listed kinematic chains as support. Return false if unknown chains present.")
	.arg("chains", "List of kinematic chains to mark as support. Others chains is marked free.");
  // Load robot model service
  robot_model_ = getProvider<RobotModel>("robot_model"); // It tries to load the service if it is not loaded.
  this->log(INFO) << "Agregator is constructed." <<endlog();
}

bool Agregator::configureHook()
{
  // check if robot model is valid
  if (!robot_model_ || !robot_model_->configure()) return false;
  // get list of joint groups
  output_support_state_.name = robot_model_->listChains();
  int n_chains = output_support_state_.name.size();
  // construct joint group index
  chain_index_.clear();
  for(int i = 0; i < n_chains; i++) 
	  chain_index_[output_support_state_.name[i]] = i;
  // get list of all joint names
  output_joint_state_.name = robot_model_->listJoints("");
  int n_joints = output_joint_state_.name.size();
  // construct joint index
  joint_index_.clear();
  for(int i = 0; i < n_joints; i++) 
	  joint_index_[output_joint_state_.name[i]] = i;
  // reserve memory
  output_joint_state_.position.assign(n_joints, 0.0);
  output_joint_state_.velocity.assign(n_joints, 0.0);
  output_joint_state_.effort.assign(n_joints, 0.0);
  output_support_state_.support.assign(n_chains, 0.0);
  // set data samples
  output_port_joint_state_.setDataSample(output_joint_state_);
  output_port_support_state_.setDataSample(output_support_state_);

  this->log(INFO) << "Agregator is configured." << endlog();
  return true;
}


bool Agregator::setSupportState(std::vector<string> limbs)
{
	if (!this->isConfigured()) return false;
	// mark all chains as free	
	output_support_state_.support.assign(output_support_state_.support.size(), 0.0);
	// mark metioned chains as support
	bool success = true;
	for ( std::string& name : limbs ) {
		auto it = chain_index_.find(name);
		if (it != chain_index_.end()) {
			output_support_state_.support[it->second] = 1.0;
		}
		else success = false;
	}
	//publish it
	output_port_support_state_.write(output_support_state_);
	return success;
}

bool Agregator::startHook()
{
  RTT::os::Timer::TimerId timer_id;
  sync_port_.readNewest(timer_id);
  // get samples
  input_port_joint_state_.getDataSample(input_joint_state_);
  input_port_support_state_.getDataSample(input_support_state_);
  // write support state
  output_port_support_state_.write(output_support_state_);

  this->log(INFO) << "Agregator is started." <<endlog();
  return true;
}

void Agregator::updateHook(){

  bool publish_joint_state=false; // Indicate that there messages to send.
  // Check if pose was updated
  int req_count = 0;
  while ( (input_port_joint_state_.read(input_joint_state_, false) == NewData) and (req_count < max_requests_per_cycle) )
  {
    req_count++;
    if (!isValidJointStateNamePos(input_joint_state_)) continue; // Message check failed. Go to next.

    publish_joint_state = publish_on_event_;

    for(int j = 0; j < input_joint_state_.name.size(); j++)
    {
      auto it = joint_index_.find(input_joint_state_.name[j]);
      if (it == joint_index_.end()) continue; // Joint not found in message

      // copy position data
      output_joint_state_.position[it->second] = input_joint_state_.position[j];
      // copy non empty velocity data
      if(input_joint_state_.velocity.size() == input_joint_state_.name.size())
        output_joint_state_.velocity[it->second] = input_joint_state_.velocity[j];
      // copy non empty effort data
      //if(input_joint_state_.effort.size() == input_joint_state_.name.size())
        //output_joint_state_.effort[it->second] = input_joint_state_.effort[j];
    }
  } 

  // Check sync port
  RTT::os::Timer::TimerId timer_id;
  if (sync_port_.read(timer_id) == NewData) publish_joint_state = publish_on_timer_;

  bool publish_support_state = false;
  // Check if support state is updated
  req_count = 0;
  while ( (input_port_support_state_.read(input_support_state_, false) == NewData) and (req_count < max_requests_per_cycle) )
  {
    req_count++;
	if (input_support_state_.name.size() != input_support_state_.support.size()) continue; // Message is not valid. Go next.

    for(int j = 0; j < input_support_state_.name.size(); j++)
    {
      auto it = chain_index_.find(input_support_state_.name[j]);
      if (it == chain_index_.end()) continue; // Unknown joint group

      output_support_state_.support[it->second] = input_support_state_.support[j];
	}
	publish_support_state = true; // Support state is updated, publish it.
  }

  // Publish messages
  if (publish_support_state) output_port_support_state_.write(output_support_state_);
  if (publish_joint_state) {
	// Set message timestamp
	output_joint_state_.header.stamp = ros::Time::now();
	output_port_joint_state_.write(output_joint_state_);
  }
}

void Agregator::stopHook() {
  this->log(INFO) << "Agregator is stopped." <<endlog();
}

void Agregator::cleanupHook() {
  this->log(INFO) << "Agregator cleaning up." <<endlog();
}

} // namespace motion
} // namespace sweetie_bot

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(sweetie_bot::Agregator)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(sweetie_bot::motion::Agregator)
