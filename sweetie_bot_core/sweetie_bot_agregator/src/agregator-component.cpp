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

  this->ports()->addEventPort( input_port_joint_state_ )
   .doc( "Messages received on this port is used to update full robot pose buffered by component. Only present fields are updated." );
  this->ports()->addPort( output_port_joint_state_ )
   .doc( "Port publishes full robot pose buffered by component. It is sorted by kinematics chains." );

  this->ports()->addEventPort( sync_port_ )
   .doc("Timer event indicating beginig of next control cycle.");

  robot_model_ = getProvider<RobotModel>("robot_model"); // It tries to load the service if it is not loaded.
  robot_model_interface_ = boost::dynamic_pointer_cast<RobotModelInterface>(this->provides()->getService("robot_model"));
  this->log(INFO) << "constructed." <<endlog();
}

bool Agregator::configureHook(){
  if((nullptr == robot_model_) or (nullptr == robot_model_interface_)) return false;
  if(!robot_model_->configure()) return false;
  chain_names_ = robot_model_->listChains();
  for(auto &name: chain_names_){
    vector<string> joints = robot_model_->listJoints(name);
    joint_names_.insert( joint_names_.end(), joints.begin(), joints.end() );
  }
  output_joint_state_.name = joint_names_;
  output_joint_state_.position.assign(joint_names_.size(), 0.0);
  output_joint_state_.velocity.assign(joint_names_.size(), 0.0);
  output_joint_state_.effort.assign(joint_names_.size(), 0.0);
  this->log(INFO) << "configured." <<endlog();
  return true;
}

bool Agregator::startHook(){
  RTT::os::Timer::TimerId timer_id;
  sync_port_.readNewest(timer_id);
  this->log(INFO) << "started." <<endlog();
  return true;
}

void Agregator::updateHook(){

  if( input_port_joint_state_.read(input_joint_state_, false) == NewData )
  {
    int req_count = 0;
    do
    {
      req_count++;
      if(!isValidJointStateNamePos(input_joint_state_)) continue; // Message check failed. Go to next.

      for(int i=0; i<joint_names_.size(); i++)
      {
        auto it = find(input_joint_state_.name.begin(), input_joint_state_.name.end(), joint_names_[i]);
        if(it == input_joint_state_.name.end()) continue;
        int j = distance(input_joint_state_.name.begin(), it);
        // copy non empty position data
        if(input_joint_state_.position.size() == input_joint_state_.name.size())
          output_joint_state_.position[i] = input_joint_state_.position[j];
        // copy non empty velocity data
        if(input_joint_state_.velocity.size() == input_joint_state_.name.size())
          output_joint_state_.velocity[i] = input_joint_state_.velocity[j];
        // copy non empty effort data
        if(input_joint_state_.effort.size() == input_joint_state_.name.size())
          output_joint_state_.effort[i] = input_joint_state_.effort[j];
      }

    } while ( (input_port_joint_state_.read(input_joint_state_, false) == NewData) and (req_count < max_requests_per_cycle) );

    // Set message timestamp
    output_joint_state_.header.stamp = ros::Time(((double)RTT::os::TimeService::Instance()->getNSecs())*1E-9);
    // Send message
    output_port_joint_state_.write(output_joint_state_);
  }

  //*
  RTT::os::Timer::TimerId timer_id;

  if (sync_port_.read(timer_id) == NewData) {
	// Set message timestamp
	output_joint_state_.header.stamp = ros::Time(((double)RTT::os::TimeService::Instance()->getNSecs())*1E-9);
	// Send pose every time cycle
	output_port_joint_state_.write(output_joint_state_);
	return; // to prevent sending message twice
  }
  // */

}

void Agregator::stopHook() {
  this->log(INFO) << "stoped." <<endlog();
}

void Agregator::cleanupHook() {
  this->log(INFO) << "cleaning up." <<endlog();
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
