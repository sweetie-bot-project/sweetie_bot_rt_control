#include <rtt/RTT.hpp>
#include <rtt/plugin/ServicePlugin.hpp>

#include "ros/ros.h"
#include <ros/console.h>
#include <rtt_rosparam/rosparam.h>

#include <kdl_parser/kdl_parser.hpp>
#include <sensor_msgs/JointState.h>
#include <kdl/jntarray.hpp>

using namespace RTT;
using namespace KDL;
using namespace std;

/**
 * Robot model service which can be loaded in a component.
 */
class RobotModelService : public Service {
private:
  std::string robot_description_;
  const std::string urdf_param_name_ = "robot_description";
  KDL::Tree tree_;
  map<string,Chain *> chains_;
  map<string,vector<string> > joints_;
  TaskContext* owner_;
public:
    RobotModelService(TaskContext* owner) 
        : Service("robot_model_service", owner),
          owner_( owner) 
    {
	this->addOperation("getOwnerName", &RobotModelService::getOwnerName, this).doc("Returns the name of the owner of this object.");
	this->provides()->addOperation("configure", &RobotModelService::configure, this, OwnThread).doc("Configires service: read parameters, construct kdl tree.");
	this->provides()->addOperation("listChains", &RobotModelService::listChains, this, ClientThread).doc("Lists loaded chains");
	this->provides()->addOperation("extractChain", &RobotModelService::extractChain, this, ClientThread).doc("Extracts chain parameters from joint state");
	this->provides()->addOperation("packChain", &RobotModelService::packChain, this, ClientThread).doc("Put chain parameters to joint state");
	owner->addProperty(urdf_param_name_, robot_description_);
	this->provides()->addProperty(urdf_param_name_, robot_description_); // not working :(
    }

    ~RobotModelService()
    {
	cleanup();
    }

    bool isConfigured()
    {
	return (robot_description_.length() > 0) and (tree_.getNrOfJoints() > 0) and (tree_.getNrOfSegments() > 0);
    }

    bool configure()
    {
	std::cout << "RobotModelService executes configure !" <<std::endl;
	if(isConfigured()){
	  std::cout << "RobotModelService already configured !" <<std::endl;
	  return true;
	}

	// Get the rosparam service requester
	boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
	  owner_->getProvider<rtt_rosparam::ROSParam>("rosparam");

	// Get the parameters
	if(!rosparam) {
	  std::cout << "RobotModelService can't get rosparam service requester !" <<std::endl;
	  return false;
	}

	// Get the ROS parameter "/robot_description"
	if(!rosparam->getAbsolute( urdf_param_name_ ))
	{
	  std::cout << "RobotModelService can't get " << urdf_param_name_ << " parameter !" <<std::endl;
          return false;
	}

	if (!kdl_parser::treeFromString(robot_description_, tree_)){
	  std::cout << "RobotModelService failed to construct kdl tree !" <<std::endl;
	  return false;
	}

	if(!readChains()){
	  std::cout << "RobotModelService can't get chains !" <<std::endl;
	  return false;
	}

	return true;
    }

    void cleanup()
    {
	std::cout << "RobotModelService cleanup !" <<std::endl;
	robot_description_ = "";
	tree_ = KDL::Tree();
    }

    bool readChains()
    {
        Property<PropertyBag> chains_bag = owner_->properties()->find("chains");
	std::cout << "readChains"<< std::endl;
        if ( !chains_bag.ready() ) return false;

	Property<std::string> first_link, last_link;

	std::cout << "chains:"<< std::endl;
	for(int i = 0; i < chains_bag.rvalue().size(); i++) {
		Property<PropertyBag> chain_bag = chains_bag.rvalue().getItem(i);
		if (!chains_bag.ready()) return false;

		std::cout << "  "<< chain_bag.getName() <<  std::endl;

		first_link = chain_bag.rvalue().getProperty("first_link");
		if (!first_link.ready()) return false;

		std::cout << "    " << first_link.getName() << ": " << first_link.rvalue() <<  std::endl;

		last_link = chain_bag.rvalue().getProperty("last_link");
		if (!last_link.ready()) return false;

		std::cout << "    " << last_link.getName() << ": " << last_link.rvalue() <<  std::endl;

		KDL::Chain chain;
		if(!tree_.getChain( first_link, last_link, chain )) return false;
		chains_[ chain_bag.getName() ] = &chain;
		// fill up joints list for each chain
		for(int i = 0; i < chain.getNrOfSegments(); i++)
		  joints_[ chain_bag.getName() ].push_back( chain.getSegment(i).getJoint().getName() );

	}
	return true;
    }

    vector<string> listChains()
    {
	vector<string> chains_list;
	for(auto iterator: chains_) {
	  chains_list.push_back(iterator.first);
	}
	return chains_list;
    }

    bool extractChain(const string& name, const sensor_msgs::JointState& joint_state, JntArray& position, JntArray& velocity, JntArray& effort)
    {
	if( ( joint_state.name.size() != joint_state.position.size() ) ||
	    ( joint_state.name.size() != joint_state.velocity.size() ) ||
	    ( joint_state.name.size() != joint_state.effort.size() ) ) return false;

	auto iterator = joints_.find(name);
	if ( iterator == joints_.end() ) {
	  // limb not found
	  return false;
	} else {
	  // found
	  const vector<string>& joints = iterator->second;

	  position.resize( joints.size() );
	  velocity.resize( joints.size() );
	  effort.resize(   joints.size() );

	  for(auto& joint: joints)
	  {
		auto j_index = &joint - &joints[0]; // joint index
		auto it = std::find(joint_state.name.begin(), joint_state.name.end(), joint);
		if (it == joint_state.name.end())
		{
		  // joint not found
		  return false;
		}
		else
		{
		  auto js_index = std::distance(joint_state.name.begin(), it); // joint_state index
		  position(j_index) = joint_state.position[js_index];
		  velocity(j_index) = joint_state.velocity[js_index];
		    effort(j_index) = joint_state.effort[js_index];
		}
	  }
	}
	return true;
    }

    bool packChain(const string& name, JntArray& position, JntArray& velocity, JntArray& effort, sensor_msgs::JointState& joint_state)
    {
        auto iterator = joints_.find(name);
        if ( iterator == joints_.end() ) {
          // limb not found
          return false;
        } else {
          // found
          const vector<string>& joints = iterator->second;

	  if( ( position.rows() != joints.size() ) || 
              ( position.rows() != velocity.rows() ) || 
              ( position.rows() != effort.rows() ) ) return false;

	  for(auto& joint: joints)
	  {
		auto j_index = &joint - &joints[0]; // joint index
		joint_state.name.push_back( joint );
		joint_state.position.push_back( position(j_index) );
		joint_state.velocity.push_back( velocity(j_index) );
		  joint_state.effort.push_back( effort(j_index) );
	  }
	}
	return true;
    }

    string getOwnerName() {
        // getOwner() returns the TaskContext pointer we got in
        // the constructor:
        return getOwner()->getName();
    }
};

ORO_SERVICE_NAMED_PLUGIN(RobotModelService, "robot_model_service")
