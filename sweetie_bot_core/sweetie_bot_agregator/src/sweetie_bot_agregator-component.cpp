#include "sweetie_bot_agregator-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

Sweetie_bot_agregator::Sweetie_bot_agregator(std::string const& name) : TaskContext(name),
                                                                          input_port_joint_state_("input_joint_state"),
                                                                          output_port_joint_state_("output_joint_state")
{

  std::cout << "Sweetie_bot_agregator constructed !" <<std::endl;
  this->ports()->addEventPort( input_port_joint_state_ ).doc( "Input port for JointState data" );
  this->ports()->addPort( output_port_joint_state_ ).doc( "Output port for JointState data" );
  robot_model_ = getProvider<RobotModel>("robot_model"); //попытается загрузить нужный сервис, если он отсутсвует.
  robot_model_interface_ = boost::dynamic_pointer_cast<RobotModelInterface>(this->provides()->getService("robot_model"));
}

bool Sweetie_bot_agregator::configureHook(){
  if((nullptr == robot_model_) or (nullptr == robot_model_interface_)) return false;
  if(!robot_model_->configure()) return false;
  chain_names_ = robot_model_->listChains();
  for(auto &name: chain_names_){
    vector<string> joints = robot_model_->listJoints(name);
    joint_names_.insert( joint_names_.end(), &joints[0], &joints[0]+joints.size() );
  }
  for(auto &name: joint_names_) {
	output_joint_state_.name.push_back(name);
	output_joint_state_.position.push_back(0.0);
	output_joint_state_.velocity.push_back(0.0);
	output_joint_state_.effort.push_back(0.0);
  }
  std::cout << "Sweetie_bot_agregator configured !" <<std::endl;
  return true;
}

bool Sweetie_bot_agregator::startHook(){
  std::cout << "Sweetie_bot_agregator started !" <<std::endl;
  return true;
}

void Sweetie_bot_agregator::updateHook(){
  std::cout << "Sweetie_bot_agregator executes updateHook !" <<std::endl;
  sensor_msgs::JointState input_joint_state;

  if( input_port_joint_state_.read(input_joint_state) == NewData ){
    for(int i=0; i<joint_names_.size(); i++){
	auto it = find(input_joint_state.name.begin(), input_joint_state.name.end(), joint_names_[i]);
	if(it == input_joint_state.name.end()) continue;
	int j = distance(input_joint_state.name.begin(), it);
	if(input_joint_state.position.size() == input_joint_state.name.size())
	  output_joint_state_.position[i] = input_joint_state.position[j];
	if(input_joint_state.velocity.size() == input_joint_state.name.size())
	  output_joint_state_.velocity[i] = input_joint_state.effort[j];
	if(input_joint_state.effort.size() == input_joint_state.name.size())
	  output_joint_state_.effort[i] = input_joint_state.effort[j];
    }
    cout << output_joint_state_ << endl;
    output_port_joint_state_.write(output_joint_state_);
  }
}

void Sweetie_bot_agregator::stopHook() {
  std::cout << "Sweetie_bot_agregator executes stopping !" <<std::endl;
}

void Sweetie_bot_agregator::cleanupHook() {
  std::cout << "Sweetie_bot_agregator cleaning up !" <<std::endl;
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(Sweetie_bot_agregator)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(Sweetie_bot_agregator)
