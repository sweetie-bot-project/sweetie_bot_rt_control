#include "sweetie_bot_agregator-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

namespace sweetie_bot {

Agregator::Agregator(std::string const& name) : TaskContext(name),
                                                                          input_port_joint_state_("in_joints"),
                                                                          output_port_joint_state_("out_joints_sorted")
{

  std::cout << "sweetie_bot::Agregator constructed !" <<std::endl;
  this->ports()->addEventPort( input_port_joint_state_ ).doc( "Input port for JointState data" );
  this->ports()->addPort( output_port_joint_state_ ).doc( "Output port for JointState data" );
  robot_model_ = getProvider<RobotModel>("robot_model"); //попытается загрузить нужный сервис, если он отсутсвует.
  robot_model_interface_ = boost::dynamic_pointer_cast<RobotModelInterface>(this->provides()->getService("robot_model"));
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
  std::cout << "sweetie_bot::Agregator configured !" <<std::endl;
  return true;
}

bool Agregator::startHook(){
  std::cout << "sweetie_bot::Agregator started !" <<std::endl;
  return true;
}

void Agregator::updateHook(){
  //std::cout << "sweetie_bot::Agregator executes updateHook !" <<std::endl;

  if( input_port_joint_state_.read(input_joint_state_) == NewData ){
    for(int i=0; i<joint_names_.size(); i++){
	auto it = find(input_joint_state_.name.begin(), input_joint_state_.name.end(), joint_names_[i]);
	if(it == input_joint_state_.name.end()) continue;
	int j = distance(input_joint_state_.name.begin(), it);
	if(input_joint_state_.position.size() == input_joint_state_.name.size())
	  output_joint_state_.position[i] = input_joint_state_.position[j];
	if(input_joint_state_.velocity.size() == input_joint_state_.name.size())
	  output_joint_state_.velocity[i] = input_joint_state_.effort[j];
	if(input_joint_state_.effort.size() == input_joint_state_.name.size())
	  output_joint_state_.effort[i] = input_joint_state_.effort[j];
    }
    // Set message timestamp
    output_joint_state_.header.stamp = ros::Time(((double)RTT::os::TimeService::Instance()->getNSecs())*1E-9);
    //cout << output_joint_state_ << endl;
    output_port_joint_state_.write(output_joint_state_);
  }
}

void Agregator::stopHook() {
  std::cout << "sweetie_bot::Agregator executes stopping !" <<std::endl;
}

void Agregator::cleanupHook() {
  std::cout << "sweetie_bot::Agregator cleaning up !" <<std::endl;
}

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
ORO_CREATE_COMPONENT(sweetie_bot::Agregator)
