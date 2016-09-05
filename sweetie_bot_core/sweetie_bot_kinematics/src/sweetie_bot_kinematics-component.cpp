#include "sweetie_bot_kinematics-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <kdl_conversions/kdl_msg.h>


Sweetie_bot_kinematics::Sweetie_bot_kinematics(std::string const& name) : TaskContext(name),
									  input_port_joints_seed_("in_joints_seed_sorted"),
									  input_port_joints_("in_joints_sorted"),
									  input_port_limbs_("in_limbs"),
									  output_port_joints_("out_joints"),
									  output_port_limbs_("out_limbs"){
  std::cout << "Sweetie_bot_kinematics constructed !" <<std::endl;
  this->ports()->addEventPort( input_port_joints_seed_ ).doc( "Input port for initial pose (JointState)" );
  this->ports()->addEventPort( input_port_joints_ ).doc( "Input port for JointState data" );
  this->ports()->addEventPort( input_port_limbs_ ).doc( "Input port for LimbState data" );
  this->ports()->addPort( output_port_joints_ ).doc( "Output port for JointState data" );
  this->ports()->addPort( output_port_limbs_ ).doc( "Output port for LimbState data" );

  //this->addOperation("", &RobotModelService::configure, this, OwnThread).doc("Configires service: read parameters, construct kdl tree.");
  robot_model_ = getProvider<RobotModel>("robot_model"); //попытается загрузить нужный сервис, если он отсутсвует.
  robot_model_interface_ = boost::dynamic_pointer_cast<RobotModelInterface>(this->provides()->getService("robot_model"));
}

bool Sweetie_bot_kinematics::configureHook(){
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
  }

  cout << "Sweetie_bot_kinematics configured !!" <<endl;
  return true;
}

bool Sweetie_bot_kinematics::startHook(){
  std::cout << "Sweetie_bot_kinematics started !" <<std::endl;
  return true;
}

void Sweetie_bot_kinematics::updateHook(){
  //std::cout << "Sweetie_bot_kinematics executes updateHook !" <<std::endl;
  cout << "Sweetie_bot_kinematics executes updateHook !" <<endl;
  sensor_msgs::JointState input_joint_seed;
  sensor_msgs::JointState input_joint_state;
  sensor_msgs::JointState output_joint_state;
  sweetie_bot_kinematics_msgs::LimbState input_limb_state;
  sweetie_bot_kinematics_msgs::LimbState output_limb_state;

  // Init seed
  if( input_port_joints_seed_.read(input_joint_seed) == NewData){
    if(input_joint_seed.name.size() == input_joint_seed.position.size())
    {
      for(auto &name: chain_names_)
      {
        JntArray position, velocity, effort;
        if(!robot_model_->extractChain(name, input_joint_seed, position, velocity, effort)) return;
	*limb_[name].seed = position;
      }
    }
  }

  // DK
  if( input_port_joints_.read(input_joint_state) == NewData ){
    for(auto &name: chain_names_) {

	KDL::Frame result_frame;
	geometry_msgs::Pose result_pose;

	JntArray position, velocity, effort;
	if(!robot_model_->extractChain(name, input_joint_state, position, velocity, effort)) return;
	int kinematics_status = limb_[name].fk_solver->JntToCart( position, result_frame );
	if(kinematics_status >= 0) {
	  tf::poseKDLToMsg(result_frame, result_pose);
	  output_limb_state.name.push_back( name );
	  output_limb_state.pose.push_back( result_pose );
	  // TODO: output_limb_state.speed.push_back(?)
	}
	else
	{
	  std::cout << "!kinematics_status=" << kinematics_status <<std::endl;
	  return;
	}
    }
    cout << output_limb_state << endl;
    output_port_limbs_.write(output_limb_state);
  }

  // IK
  if( input_port_limbs_.read(input_limb_state) == NewData ){
    for(auto &name: chain_names_) {
	auto it = find(input_limb_state.name.begin(), input_limb_state.name.end(), name);
	if(it == input_limb_state.name.end()) continue;
	int limb_num = distance(input_limb_state.name.begin(), it);

	KDL::Frame desired_end_effector_pose;
	tf::poseMsgToKDL( input_limb_state.pose[limb_num], desired_end_effector_pose );
	KDL::JntArray return_joints_positions, zero_joints_speed, zero_joints_effort;
	int kinematics_status = limb_[name].ik_solver->CartToJnt(*limb_[name].seed, desired_end_effector_pose, return_joints_positions); //, tolerances);
	if(kinematics_status >= 0) {
		robot_model_->packChain( name, return_joints_positions, zero_joints_speed, zero_joints_effort, output_joint_state);
	}
        else
        {
          std::cout << "!kinematics_status=" << kinematics_status <<std::endl;
	  return;
        }
    }
    cout << output_joint_state << endl;
    output_port_joints_.write(output_joint_state);
  }

}

void Sweetie_bot_kinematics::stopHook() {
  std::cout << "Sweetie_bot_kinematics executes stopping !" <<std::endl;
}

void Sweetie_bot_kinematics::cleanupHook() {
  std::cout << "Sweetie_bot_kinematics cleaning up !" <<std::endl;
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(Sweetie_bot_kinematics)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(Sweetie_bot_kinematics)
