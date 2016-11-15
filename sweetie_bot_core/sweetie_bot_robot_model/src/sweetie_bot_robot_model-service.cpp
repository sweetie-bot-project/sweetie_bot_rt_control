#include <unordered_map>

#include <rtt/RTT.hpp>
#include <rtt/plugin/ServicePlugin.hpp>

#include <sweetie_bot_robot_model/sweetie_bot_robot_model-requester.hpp>

#include <kdl_parser/kdl_parser.hpp>

#include <Eigen/Dense>

using namespace std;
using namespace RTT;
using namespace KDL;
using namespace Eigen;

namespace sweetie_bot {

/**
 * Robot model service which can be loaded in a component.
 */
class RobotModelService : public RobotModelInterface, public Service {
private:
  std::string robot_description_;
  KDL::Tree tree_;
  unordered_map<string,Chain*> chains_;
  unordered_map<string,pair<char, char>> chain_pos_;
  vector<string> chain_names_;
  vector<string> joint_names_;
  TaskContext* owner_;
public:
    RobotModelService(TaskContext* owner) 
        : Service("robot_model", owner),
          owner_( owner) 
    {
	this->addOperation("getOwnerName", &RobotModelService::getOwnerName, this).doc("Returns the name of the owner of this object.");
	this->addOperation("configure", &RobotModelService::configure, this, OwnThread).doc("Configures service: read parameters, construct kdl tree.");
	this->addOperation("listChains", &RobotModelService::listChains, this, ClientThread).doc("Lists loaded chains");
	this->addOperation("listJoints", &RobotModelService::listJoints, this, ClientThread).doc("Lists joints in chain");
	this->addOperation("listAllJoints", &RobotModelService::listAllJoints, this, ClientThread).doc("Lists all joints in all chains");
	this->addOperation("getJointPos", &RobotModelService::getJointPos, this).doc("Returns position of the given joint name in sorted pose.");
	this->addOperation("extractChain", &RobotModelService::extractChain, this, ClientThread).doc("Extracts (copy) chain parameters from joint state");
	this->addOperation("packChain", &RobotModelService::packChain, this, ClientThread).doc("Put chain parameters to joint state");
	this->addProperty("robot_description", robot_description_);
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
	for(auto &chain: chains_){
	  free(chain.second);
	}
	robot_description_ = "";
	tree_ = KDL::Tree();
    }

    bool readChains()
    {
        Property<PropertyBag> chains_bag = owner_->properties()->find("chains");
        if ( !chains_bag.ready() ) return false;

	Property<std::string> first_link, last_link;
	char joint_num = 0;
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
		KDL::Chain * chain = new KDL::Chain();
		if(!tree_.getChain( first_link, last_link, *chain )) return false;

		chain_names_.push_back(chain_bag.getName());
		chains_.insert( make_pair( chain_bag.getName(), chain ));
		chain_pos_[ chain_bag.getName() ] = { joint_num, chain->getNrOfSegments() };

		// fill up joints list for each chain
		for(int j = 0; j < chain->getNrOfSegments(); j++){
		  string joint_name = chain->getSegment(j).getJoint().getName();
		  std::cout << "      joint(" << (int)joint_num << ")=" << joint_name << std::endl;
		  joint_num++;
		  joint_names_.push_back(joint_name);
		}

	}
	return true;
    }

    vector<string> listChains()
    {
	return chain_names_;
    }

    vector<string> listJoints(const string& name)
    {
	vector<string> joint_names;
	Chain * chain = getChain(name);
	if( chain != nullptr ) {
	  for(int j = 0; j < chain->getNrOfSegments(); j++){
	    joint_names.push_back( chain->getSegment(j).getJoint().getName() );
	  }
 	}
	return joint_names;
    }

    vector<string> listAllJoints()
    {
	return joint_names_;
    }

    int getJointPos(const string& name)
    {
	auto it = find(joint_names_.begin(), joint_names_.end(), name);
	return (it == joint_names_.end()) ? -1 : distance(joint_names_.begin(), it);
    }

    Chain * getChain(const string& name)
    {
	auto iterator = chains_.find(name);
	if ( iterator == chains_.end() ) {
          // limb not found
	  return nullptr;
	}
	else { 
	  // found
	  return iterator->second;
	}
    }

    bool mapChain(const string& name, sensor_msgs::JointState& joint_state, JntArray& position, JntArray& velocity, JntArray& effort)
    {
        if(!equal( joint_names_.begin(), joint_names_.end(), joint_state.name.begin() )) return false;
	char chain_begin, chain_size;
	tie(chain_begin, chain_size) = chain_pos_.at(name);
	if(joint_state.name.size() == joint_state.position.size() ) {
	  // map buffer to JntddArray
	  new (&position.data) Eigen::Map<VectorXd>( &joint_state.position[ chain_begin ], chain_size );
	}
	if(joint_state.name.size() == joint_state.velocity.size() ) {
	  // map buffer to JntArray
	  new (&velocity.data) Eigen::Map<VectorXd>( &joint_state.velocity[ chain_begin ], chain_size );
	}
	if(joint_state.name.size() == joint_state.effort.size() ) {
	  // map buffer to JntArray
	  new (&effort.data)   Eigen::Map<VectorXd>( &joint_state.effort  [ chain_begin ], chain_size );
	}
	return true;
    }

    bool extractChain(const string& name, const sensor_msgs::JointState& joint_state, JntArray& position, JntArray& velocity, JntArray& effort)
    {
        if(!equal( joint_names_.begin(), joint_names_.end(), joint_state.name.begin() )) return false;
	char chain_begin, chain_size;
	tie(chain_begin, chain_size) = chain_pos_.at(name);
	if(joint_state.name.size() == joint_state.position.size() ) {
	  // copy buffer to JntArray
	  position.data = VectorXd::Map( &joint_state.position[ chain_begin ], chain_size );
	}
	if(joint_state.name.size() == joint_state.velocity.size() ) {
	  // copy buffer to JntArray
	  velocity.data = VectorXd::Map( &joint_state.velocity[ chain_begin ], chain_size );
	}
	if(joint_state.name.size() == joint_state.effort.size() ) {
	  // copy buffer to JntArray
	  effort.data = VectorXd::Map( &joint_state.effort  [ chain_begin ], chain_size );
	}
	return true;
    }

    bool packChain(const string& name, JntArray& position, JntArray& velocity, JntArray& effort, sensor_msgs::JointState& joint_state)
    {
	Chain chain;
	if(!equal( joint_names_.begin(), joint_names_.end(), joint_state.name.begin() )) return false;

	char chain_begin, chain_size;
	tie(chain_begin, chain_size) = chain_pos_.at(name);

	if(( position.rows() > 0 ) and ( position.rows() == chain_size )) {
	  std::copy_n( &position.data[0], chain_size, joint_state.position.begin()+chain_begin );
	}
	if(( velocity.rows() > 0 ) and ( velocity.rows() == chain_size )) {
	  std::copy_n( &velocity.data[0], chain_size, joint_state.velocity.begin()+chain_begin );
	}
	if(( effort.rows() > 0 ) and ( effort.rows() == chain_size )) {
	  std::copy_n( &effort.data[0],   chain_size, joint_state.effort.begin()+chain_begin );
	}
	return true;
    }

    string getOwnerName() {
        // getOwner() returns the TaskContext pointer we got in
        // the constructor:
        return getOwner()->getName();
    }
};

} // namespace sweetie_bot

ORO_SERVICE_NAMED_PLUGIN(sweetie_bot::RobotModelService, "robot_model")
