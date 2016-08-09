#include "urdf_need-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

Urdf_need::Urdf_need(std::string const& name) : TaskContext(name),
						inJointState("input_joint_state"){
  std::cout << "Urdf_need constructed !" <<std::endl;
  this->ports()->addEventPort( inJointState ).doc( "Input port for JointState data" );
}

bool Urdf_need::configureHook(){
  std::cout << "Urdf_need configured !" <<std::endl;
  robot_model_req = getProvider<RobotModel>("robot_model_service"); //попытается загрузить нужный сервис, если он отсутсвует.
//  bool rez;
//  if(!robot_model_req) return false;
//    rez = robot_model_req->configure();
  return robot_model_req != 0 and robot_model_req->configure();
  //return true;
}

bool Urdf_need::startHook(){
  std::cout << "Urdf_need started !" <<std::endl;
  return true;
}

void Urdf_need::updateHook(){
  std::cout << "Urdf_need executes updateHook !" <<std::endl;
  //getProvider<RobotModel>("robot_model_service")->configure();
  sensor_msgs::JointState input_joint_state;
  JntArray position;
  JntArray speed;
  JntArray effort;
  switch ( inJointState.read(input_joint_state) )
  {
    case NoData:  std::cout << "NO data received !" <<std::endl;
                  return;
    case OldData: std::cout << "OLD data received !" <<std::endl;
                  return;
    case NewData: std::cout << "New data received !" <<std::endl;
                  break;
  }
  if(!robot_model_req->extractChain("leg3", input_joint_state, position, speed, effort))
  {
	std::cout << "Urdf_need extractChain false!" <<std::endl;
  }
  std::cout << position.data << std::endl;
  sensor_msgs::JointState joint_state;
  if(!robot_model_req->packChain("leg3", position, speed, effort, joint_state))
  {
	std::cout << "Urdf_need packChain false!" <<std::endl;
  }
  std::cout << joint_state <<std::endl;
}

void Urdf_need::stopHook() {
  std::cout << "Urdf_need executes stopping !" <<std::endl;
}

void Urdf_need::cleanupHook() {
  std::cout << "Urdf_need cleaning up !" <<std::endl;
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(Urdf_need)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(Urdf_need)
