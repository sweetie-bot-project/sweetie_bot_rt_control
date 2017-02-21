#include "kinematics-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <kdl_conversions/kdl_msg.h>

using sweetie_bot::logger::Logger;
using namespace RTT;
using namespace KDL;

namespace sweetie_bot {
namespace motion {

Kinematics::Kinematics(string const& name) : TaskContext(name),
									  input_port_joints_seed_("in_joints_seed_sorted"),
									  input_port_joints_("in_joints_sorted"),
									  input_port_limbs_("in_limbs"),
									  output_port_joints_("out_joints"),
									  output_port_limbs_("out_limbs"),
									  log(logger::getDefaultCategory("sweetie_bot.motion") + "." + name)
{
  this->ports()->addEventPort( input_port_joints_seed_ )
   .doc( "Messages received on this port is used to update initial robot pose used for inverse kinematic calculation." );
  this->ports()->addEventPort( input_port_joints_ )
   .doc( "Messages received on this port is used as input data for forward kinematic calculation." );
  this->ports()->addEventPort( input_port_limbs_ )
   .doc( "Messages received on this port is used as input data for inverse kinematic calculation." );
  this->ports()->addPort( output_port_joints_ )
   .doc( "Inverse kinematic results data port" );
  this->ports()->addPort( output_port_limbs_ )
   .doc( "Forward kinematic results data port" );

  robot_model_ = getProvider<RobotModel>("robot_model"); // It tries to load the service if it is not loaded.
  robot_model_interface_ = boost::dynamic_pointer_cast<RobotModelInterface>(this->provides()->getService("robot_model"));
  this->log(INFO) << "constructed." <<endlog();
}

bool Kinematics::configureHook()
{
  if((nullptr == robot_model_) or (nullptr == robot_model_interface_)) {
	this->log(ERROR) << "Can't load robot_model!" <<endlog();
	return false;
  }
  if(!robot_model_->configure()) {
	this->log(ERROR) << "Can't configure robot_model!" <<endlog();
	return false;
  }
  chain_names_ = robot_model_->listChains();
  joint_names_ = robot_model_->listAllJoints();
  for(auto &name: chain_names_) {
    Chain * chain = robot_model_interface_->getChain(name);
    vector<string> joints = robot_model_->listJoints(name);
    if(nullptr == chain) {
	this->log(ERROR) << "Can't get chain!" <<endlog();
	return false;
    }
    int nj = chain->getNrOfSegments();

    JntArray * joint_seed = new JntArray(nj);
    JntArray lower_joint_limits(nj);
    JntArray upper_joint_limits(nj);
    for(int i=0; i<nj; i++){
	lower_joint_limits(i) = -1.57;
	upper_joint_limits(i) =  1.57;
	(*joint_seed)(i) = 0;
    }

    limb_[name].chain     = make_shared<Chain>(*chain);
    limb_[name].joints    = make_shared<vector<string>>(joints);
    limb_[name].fk_solver = make_shared<ChainFkSolverPos_recursive>(*chain);
    limb_[name].ik_solver = make_shared<TRAC_IK::TRAC_IK>(*chain, lower_joint_limits, upper_joint_limits, 0.025, 1e-5, TRAC_IK::Speed);
    limb_[name].seed      = make_shared<JntArray>(*joint_seed);

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

  this->log(INFO) << "configured." <<endlog();
  return true;
}

bool Kinematics::startHook(){
  this->log(INFO) << "started." <<endlog();
  return true;
}

void Kinematics::updateHook(){
  //this->log(INFO) << "executes updateHook !" << endlog();

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

	Frame result_frame;
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
	  this->log(ERROR) << "kinematics_status=" << kinematics_status <<endlog();
	  return;
	}
    }
    // Set message timestamp
    output_limb_state_.header.stamp = ros::Time(((double)RTT::os::TimeService::Instance()->getNSecs())*1E-9);
    output_port_limbs_.write(output_limb_state_);
  }

  // IK
  if( input_port_limbs_.read(input_limb_state_) == NewData ){
    output_joint_state_.position.assign(joint_names_.size(), 0.0);
    output_joint_state_.velocity.assign(joint_names_.size(), 0.0);
    output_joint_state_.effort.assign(joint_names_.size(), 0.0);

    int limb_num = 0;
    for(auto &name: input_limb_state_.name) {
	auto it = find(chain_names_.begin(), chain_names_.end(), name);
	if(it == chain_names_.end()) {
	  this->log(ERROR) << " limb '" << name << "' doesn't exists! " <<endlog();
	  return;
	}

	Frame desired_end_effector_pose;
	tf::poseMsgToKDL( input_limb_state_.pose[limb_num], desired_end_effector_pose );
	JntArray return_joints_positions, zero_joints_speed, zero_joints_effort;
	int kinematics_status = limb_[name].ik_solver->CartToJnt(*limb_[name].seed, desired_end_effector_pose, return_joints_positions); //, tolerances);
	if(kinematics_status >= 0) {
	  if(!robot_model_->packChain( name, return_joints_positions, zero_joints_speed, zero_joints_effort, output_joint_state_)) return;
	}
        else
        {
          this->log(ERROR) << "!kinematics_status=" << kinematics_status <<endlog();
	  return;
        }
	limb_num++;
    }
    // Set message timestamp
    output_joint_state_.header.stamp = ros::Time(((double)RTT::os::TimeService::Instance()->getNSecs())*1E-9);
    output_port_joints_.write(output_joint_state_);
  }

}

void Kinematics::stopHook() {
  this->log(INFO) << "stoped." <<endlog();
}

void Kinematics::cleanupHook() {
  this->log(INFO) << "cleaning up." <<endlog();
}

} // namespace motion
} // namespace sweetie_bot

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(sweetie_bot::motion::Kinematics)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(sweetie_bot::motion::Kinematics)
