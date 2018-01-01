#include "agregator-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

using namespace std;
using namespace RTT;

namespace sweetie_bot {
namespace motion {

Agregator::Agregator(string const& name) : TaskContext(name),
	input_port_joint_state_("in_joints"),
	output_port_joint_state_("out_joints_sorted"),
	sync_port_("sync_step"),
	log(logger::categoryFromComponentName(name))
{
  // Ports 
  this->ports()->addEventPort( input_port_joint_state_ )
   .doc( "Messages received on this port is used to update full robot pose buffered by component. Only present fields are updated." );
  this->ports()->addPort( output_port_joint_state_ )
   .doc( "Port publishes full robot pose buffered by component. It is sorted by kinematics chains." );

  this->ports()->addEventPort( sync_port_ )
   .doc("Timer event indicating the beginning of next control cycle.");
  // Properties
  this->addProperty("publish_on_timer", publish_on_timer_)
    .doc("Publish robot pose when sync port message is received.").set(true);
  this->addProperty("publish_on_event", publish_on_event_)
    .doc("Publish robot pose when partial pose is received.").set(true);
  // Load robot model service
  robot_model_ = getProvider<RobotModel>("robot_model"); // It tries to load the service if it is not loaded.
  this->log(INFO) << "Agregator is constructed." <<endlog();
}

bool Agregator::configureHook()
{
  // check if robot model is valid
  if (!robot_model_ || !robot_model_->configure()) return false;
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
  this->log(INFO) << "Agregator is configured." << endlog();
  return true;
}

bool Agregator::startHook(){
  RTT::os::Timer::TimerId timer_id;
  sync_port_.readNewest(timer_id);
  this->log(INFO) << "Agregator is started." <<endlog();
  return true;
}

void Agregator::updateHook(){

  bool publish_state=false; // Indicate that there messages to send.

  // Check if pose was updated
  int req_count = 0;
  while ( (input_port_joint_state_.read(input_joint_state_, false) == NewData) and (req_count < max_requests_per_cycle) )
  {
    req_count++;
    if (!isValidJointStateNamePos(input_joint_state_)) continue; // Message check failed. Go to next.

    publish_state = publish_on_event_;

    for(int j = 0; j < input_joint_state_.name.size(); j++)
    {
      auto it = joint_index_.find(input_joint_state_.name[j]);
      if (it == joint_index_.end()) continue; // Joint not found in message

      // copy non empty position data
      if(input_joint_state_.position.size() == input_joint_state_.name.size())
        output_joint_state_.position[it->second] = input_joint_state_.position[j];
      // copy non empty velocity data
      if(input_joint_state_.velocity.size() == input_joint_state_.name.size())
        output_joint_state_.velocity[it->second] = input_joint_state_.velocity[j];
      // copy non empty effort data
      if(input_joint_state_.effort.size() == input_joint_state_.name.size())
        output_joint_state_.effort[it->second] = input_joint_state_.effort[j];
    }
  } 

  // Check sync port
  RTT::os::Timer::TimerId timer_id;
  if (sync_port_.read(timer_id) == NewData) publish_state = publish_on_timer_;

  // Publish message
  if (publish_state) {
	// Set message timestamp
	output_joint_state_.header.stamp = ros::Time(((double)RTT::os::TimeService::Instance()->getNSecs())*1E-9);
	// Send pose every time cycle
	output_port_joint_state_.write(output_joint_state_);
	return; // to prevent sending message twice
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
