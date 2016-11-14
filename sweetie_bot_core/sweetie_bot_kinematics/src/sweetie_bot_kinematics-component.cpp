#include "sweetie_bot_kinematics-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <kdl_conversions/kdl_msg.h>

namespace sweetie_bot {

Kinematics::Kinematics(std::string const& name) : TaskContext(name),
									  input_port_joints_seed_("in_joints_seed_sorted"),
									  input_port_joints_("in_joints_sorted"),
									  input_port_limbs_("in_limbs"),
									  output_port_joints_("out_joints"),
									  output_port_limbs_("out_limbs"){
  std::cout << "sweetie_bot::Kinematics constructed !" <<std::endl;
  this->ports()->addEventPort( input_port_joints_seed_ ).doc( "Input port for initial pose (JointState)" );
  this->ports()->addEventPort( input_port_joints_ ).doc( "Input port for JointState data" );
  this->ports()->addEventPort( input_port_limbs_ ).doc( "Input port for LimbState data" );
  this->ports()->addPort( output_port_joints_ ).doc( "Output port for JointState data" );
  this->ports()->addPort( output_port_limbs_ ).doc( "Output port for LimbState data" );

  robot_model_ = getProvider<RobotModel>("robot_model"); //попытается загрузить нужный сервис, если он отсутсвует.
  robot_model_interface_ = boost::dynamic_pointer_cast<RobotModelInterface>(this->provides()->getService("robot_model"));
}

bool sweetie_bot::Kinematics::configureHook(){
  if((nullptr == robot_model_) or (nullptr == robot_model_interface_)) return false;
  if(!robot_model_->configure()) return false;
  chain_names_ = robot_model_->listChains();
  joint_names_ = robot_model_->listAllJoints();
  for(auto &name: chain_names_) {
    Chain * chain = robot_model_interface_->getChain(name);
    vector<string> joints = robot_model_->listJoints(name);
    if(nullptr == chain) return false;
    int nj = chain->getNrOfSegments();

    KDL::JntArray * joint_seed = new KDL::JntArray(nj);
    KDL::JntArray lower_joint_limits(nj);
    KDL::JntArray upper_joint_limits(nj);
    for(int i=0; i<nj; i++){
	lower_joint_limits(i) = -1.57;
	upper_joint_limits(i) =  1.57;
	(*joint_seed)(i) = 0;
    }

    limb_[name].chain     = std::make_shared<Chain>(*chain);
    limb_[name].joints    = std::make_shared<vector<string>>(joints);
    limb_[name].fk_solver = std::make_shared<ChainFkSolverPos_recursive>(*chain);
    limb_[name].ik_solver = std::make_shared<TRAC_IK::TRAC_IK>(*chain, lower_joint_limits, upper_joint_limits, 0.025, 1e-5, TRAC_IK::Speed);
    limb_[name].seed      = std::make_shared<JntArray>(*joint_seed);

    // init port data
    input_joint_seed_.name = joint_names_;
    input_joint_seed_.position.resize(joint_names_.size());
    input_joint_seed_.velocity.resize(joint_names_.size());
    input_joint_seed_.effort.resize(joint_names_.size());

    input_joint_state_.name = joint_names_;
    input_joint_state_.position.resize(joint_names_.size());
    input_joint_state_.velocity.resize(joint_names_.size());
    input_joint_state_.effort.resize(joint_names_.size());

    output_joint_state_.name = joint_names_;
    output_joint_state_.position.resize(joint_names_.size());
    output_joint_state_.velocity.resize(joint_names_.size());
    output_joint_state_.effort.resize(joint_names_.size());
    output_port_joints_.setDataSample(output_joint_state_);

    input_limb_state_.name = chain_names_;
    input_limb_state_.pose.resize(chain_names_.size());
    input_limb_state_.speed.resize(chain_names_.size());

    output_limb_state_.name = chain_names_;
    output_limb_state_.pose.resize(chain_names_.size());
    output_limb_state_.speed.resize(chain_names_.size());
    output_port_limbs_.setDataSample(output_limb_state_);
  }

  cout << "sweetie_bot::Kinematics configured !" <<endl;
  return true;
}

bool sweetie_bot::Kinematics::startHook(){
  std::cout << "sweetie_bot::Kinematics started !" <<std::endl;
  return true;
}

void sweetie_bot::Kinematics::updateHook(){
  //cout << "sweetie_bot::Kinematics executes updateHook !" <<endl;

  // Init seed
  if( input_port_joints_seed_.read(input_joint_seed_) == NewData){
    if(input_joint_seed_.name.size() == input_joint_seed_.position.size())
    {
      for(auto &name: chain_names_)
      {
        JntArray position, velocity, effort;
        if(!robot_model_->extractChain(name, input_joint_seed_, position, velocity, effort)) return;
	*limb_[name].seed = position;
      }
    }
  }

  // DK
  if( input_port_joints_.read(input_joint_state_) == NewData ){
    for(auto &name: chain_names_) {

	KDL::Frame result_frame;
	geometry_msgs::Pose result_pose;

	JntArray position, velocity, effort;
	if(!robot_model_->extractChain(name, input_joint_state_, position, velocity, effort)) return;
	int kinematics_status = limb_[name].fk_solver->JntToCart( position, result_frame );
	if(kinematics_status >= 0) {
	  tf::poseKDLToMsg(result_frame, result_pose);
	  auto it = find(output_limb_state_.name.begin(), output_limb_state_.name.end(), name);
	  if(it!=output_limb_state_.name.end())
	  {
	    int i = distance(output_limb_state_.name.begin(), it);
	    output_limb_state_.pose[i] = result_pose;
	  }
	  // TODO: output_limb_state_.speed[i] = (?)
	}
	else
	{
	  std::cout << "!kinematics_status=" << kinematics_status <<std::endl;
	  return;
	}
    }
    // Set message timestamp
    output_limb_state_.header.stamp = ros::Time(((double)RTT::os::TimeService::Instance()->getNSecs())*1E-9);
    //cout << output_limb_state_ << endl;
    output_port_limbs_.write(output_limb_state_);
  }

  // IK
  if( input_port_limbs_.read(input_limb_state_) == NewData ){
    output_joint_state_.position.assign(joint_names_.size(), 0.0);
    output_joint_state_.velocity.assign(joint_names_.size(), 0.0);
    output_joint_state_.effort.assign(joint_names_.size(), 0.0);

    for(auto &name: chain_names_) {
	auto it = find(input_limb_state_.name.begin(), input_limb_state_.name.end(), name);
	if(it == input_limb_state_.name.end()) continue;
	int limb_num = distance(input_limb_state_.name.begin(), it);

	KDL::Frame desired_end_effector_pose;
	tf::poseMsgToKDL( input_limb_state_.pose[limb_num], desired_end_effector_pose );
	KDL::JntArray return_joints_positions, zero_joints_speed, zero_joints_effort;
	int kinematics_status = limb_[name].ik_solver->CartToJnt(*limb_[name].seed, desired_end_effector_pose, return_joints_positions); //, tolerances);
	if(kinematics_status >= 0) {
	  if(!robot_model_->packChain( name, return_joints_positions, zero_joints_speed, zero_joints_effort, output_joint_state_)) return;
	}
        else
        {
          std::cout << "!kinematics_status=" << kinematics_status <<std::endl;
	  return;
        }
    }
    // Set message timestamp
    output_joint_state_.header.stamp = ros::Time(((double)RTT::os::TimeService::Instance()->getNSecs())*1E-9);
    //cout << output_joint_state_ << endl;
    output_port_joints_.write(output_joint_state_);
  }

}

void sweetie_bot::Kinematics::stopHook() {
  std::cout << "sweetie_bot::Kinematics executes stopping !" <<std::endl;
}

void sweetie_bot::Kinematics::cleanupHook() {
  std::cout << "sweetie_bot::Kinematics cleaning up !" <<std::endl;
}

} // namespace sweetie_bot

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(sweetie_bot::Kinematics)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(sweetie_bot::Kinematics)
