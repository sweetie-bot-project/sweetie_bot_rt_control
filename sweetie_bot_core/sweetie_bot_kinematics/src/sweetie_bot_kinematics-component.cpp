#include "sweetie_bot_kinematics-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <kdl_conversions/kdl_msg.h>


Sweetie_bot_kinematics::Sweetie_bot_kinematics(std::string const& name) : TaskContext(name),
									  input_port_joint_state_("input_joint_state"),
									  input_port_limbs_cartesian_("input_limbs_cartesian"),
									  output_port_joint_state_("output_joint_state"),
									  output_port_limbs_cartesian_("output_limbs_cartesian"){
  std::cout << "Sweetie_bot_kinematics constructed !" <<std::endl;
  this->ports()->addEventPort( input_port_joint_state_ ).doc( "Input port for JointState data" );
  this->ports()->addEventPort( input_port_limbs_cartesian_ ).doc( "Input port for CartesianState data" );
  this->ports()->addPort( output_port_joint_state_ ).doc( "Output port for JointState data" );
  this->ports()->addPort( output_port_limbs_cartesian_ ).doc( "Output port for CartesianState data" );

  //this->addOperation("", &RobotModelService::configure, this, OwnThread).doc("Configires service: read parameters, construct kdl tree.");
  robot_model_ = getProvider<RobotModel>("robot_model"); //попытается загрузить нужный сервис, если он отсутсвует.
  robot_model_interface_ = boost::dynamic_pointer_cast<RobotModelInterface>(this->provides()->getService("robot_model"));
}

bool Sweetie_bot_kinematics::configureHook(){
  if((nullptr == robot_model_) or (nullptr == robot_model_interface_)) return false;
  if(!robot_model_->configure()) return false;
  chain_names_ = robot_model_->listChains();
  for(auto &name: chain_names_) {
    Chain * chain = robot_model_interface_->getChain(name);
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
  sensor_msgs::JointState input_joint_state;
  sensor_msgs::JointState output_joint_state;
  sweetie_bot_kinematics_msgs::CartesianState input_limbs_cartesian;
  sweetie_bot_kinematics_msgs::CartesianState output_limbs_cartesian;

  if( input_port_joint_state_.read(input_joint_state) == NewData ){
    for(auto &name: chain_names_) {

	KDL::Frame result_frame;
	geometry_msgs::Pose result_pose;

	JntArray position, velocity, effort;
	if(!robot_model_->extractChain(name, input_joint_state, position, velocity, effort)) return;
	int kinematics_status = limb_[name].fk_solver->JntToCart( position, result_frame );
	if(kinematics_status >= 0) {
	  tf::poseKDLToMsg(result_frame, result_pose);
	  output_limbs_cartesian.name.push_back( name );
	  output_limbs_cartesian.pose.push_back( result_pose );
	  // TODO: output_limbs_cartesian.speed.push_back(?)
	}
	else
	{
	  std::cout << "!kinematics_status=" << kinematics_status <<std::endl;
	  return;
	}
    }
    cout << output_limbs_cartesian << endl;
    output_port_limbs_cartesian_.write(output_limbs_cartesian);
  }

  if( input_port_limbs_cartesian_.read(input_limbs_cartesian) == NewData ){
    for(auto &name: chain_names_) {
	auto it = find(input_limbs_cartesian.name.begin(), input_limbs_cartesian.name.end(), name);
	if(it == input_limbs_cartesian.name.end()) continue;
	int limb_num = distance(input_limbs_cartesian.name.begin(), it);

	KDL::Frame desired_end_effector_pose;
	tf::poseMsgToKDL( input_limbs_cartesian.pose[limb_num], desired_end_effector_pose );
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
    output_port_joint_state_.write(output_joint_state);
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
