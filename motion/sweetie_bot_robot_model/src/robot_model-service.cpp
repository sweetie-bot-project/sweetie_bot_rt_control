#include <sweetie_bot_robot_model/robot_model-simple.hpp>

#include <unordered_map>

#include <Eigen/Dense>
#include <kdl/jntarray.hpp>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <rtt/RTT.hpp>
#include <rtt/plugin/ServicePlugin.hpp>
#include <sensor_msgs/typekit/JointState.h>

#include <sweetie_bot_logger/logger.hpp>

//#include <rtt/PropertyBag.hpp>

using namespace std;
using namespace RTT;
using namespace KDL;
using namespace Eigen;

namespace sweetie_bot {
namespace motion {

/**
 * Robot model service which can be loaded in a component.
 */
class RobotModelService : public RobotModelInterface, public Service {
	protected:
		struct ChainInfo {
			string name; // joint group name
			int index_begin; // index of the first joint in group
			int size; // chain length
			KDL::Chain kdl_chain; // KDL chain model
		};

	protected:
		// SERVICE INTERFACE
		// parameters
		std::string robot_description_;
		PropertyBag chains_prop_;
		
		// SERVICE STATE
		TaskContext* owner_;
		bool is_configured;
		// joints list
		vector<string> joint_names_;
		// joint groups
		vector<ChainInfo> chains_info_; 
		// index to speed up joint search
		map<string, int> joints_index_;
		map<string, int> chains_index_;
		// KDL kinematic model
		KDL::Tree tree_;

		// logging
#ifdef SWEETIEBOT_LOGGER
		sweetie_bot::logger::SWEETIEBOT_LOGGER log;
#else
		sweetie_bot::logger::LoggerRTT log;
#endif

	public:
		RobotModelService(TaskContext* owner) : 
			Service("robot_model", owner),
			owner_( owner),
			is_configured(false),
			log(logger::getDefaultCategory("sweetie_bot.motion") + "." + "robot_model")
		{
			this->provides()->doc("Robot model service plugin. Provides unifed access to kinematic chains and joint enumeration function");

			this->addProperty("robot_description", robot_description_)
				.doc("Robot description in URDF format");
			this->addProperty("chains", chains_prop_)
				.doc("Joint groups (kinematics chains) descriptions (PropertuBag). Format: \n"
					 "\t\t\t{\n"
					 "\t\t\t    PropertyBag chain_name1 { string first_link, string last_link },\n"
					 "\t\t\t    PropertyBag chain_name2 { ... }\n"
					 "\t\t\t    ...\n"
					 "\t\t\t}");

			this->addOperation("getOwnerName", &RobotModelService::getOwnerName, this)
				.doc("Returns this service owner name.");
			this->addOperation("configure", &RobotModelService::configure, this, OwnThread)
				.doc("Configures service: read parameters, construct kdl tree.");
			this->addOperation("getRobotDescription", &RobotModelService::getRobotDescription, this, ClientThread)
				.doc("Return robot description string (URDF model).");
			this->addOperation("listChains", &RobotModelService::listChains, this, ClientThread)
				.doc("Return the list of known kinematic chains.");
			this->addOperation("listJoints", &RobotModelService::listJoints, this, ClientThread)
				.doc("Return the list of joints in given kinematic chain. If chain name is empty return all joints.")
				.arg("chain", "Chain name.");
			this->addOperation("getJointChain", &RobotModelService::getJointChain, this, ClientThread)
				.doc("Return name of kinematic chain to which belongs given joint.")
				.arg("joint", "Joint name.");
			this->addOperation("getJointsChains", &RobotModelService::getJointsChains, this, ClientThread)
				.doc("Returns list of chains names to which the given joints are belongs.")
				.arg("joints", "List of joints names.");
			this->addOperation("getJointIndex", &RobotModelService::getJointIndex, this, ClientThread)
				.doc("Returns position (index) of the given joint in full pose sorted by chains and joints.")
				.arg("joint", "Joint name");
			this->addOperation("extractChain", &RobotModelService::extractChain, this, ClientThread)
				.doc("Extracts (copy) chain parameters from JointState message to JntArrays. Can form out_of_range exception.");
			this->addOperation("packChain", &RobotModelService::packChain, this, ClientThread)
				.doc("Compose JointState message from JntArrays. Can form out_of_range exception.");

			//this->addOperation("getChain", &RobotModelService::getChainImpl, this, ClientThread)
				//.doc("Get KDL::Chain object. Can form out_of_range exception.");
		}

		~RobotModelService()
		{
			cleanup();
		}

		bool isConfigured()
		{
			return is_configured;
		}

		bool configure()
		{
			if (is_configured) {
				// this->log(WARN) << "RobotModel is already configured!" << endlog();
				return true;
			}
			// Load URDF robot description into KDL Tree
			if (!kdl_parser::treeFromString(robot_description_, tree_)) {
				this->log(ERROR) << "Failed to construct kdl tree!" <<endlog();
				cleanup();
				return false;
			}
			// Get kinematics chains from properties
			if (!readChains()) {
				cleanup();	
				return false;
			}

			this->log(INFO) << "RobotModel is configured." <<endlog();
			is_configured = true;
			return true;
		}

		void cleanup()
		{	
			// clear all buffers
			joint_names_.clear();
			chains_info_.clear();
			joints_index_.clear();
			chains_index_.clear();
			tree_ = KDL::Tree();
			is_configured = false;
			this->log(INFO) << "RobotModel is cleaned up." <<endlog();
		}

		bool readChains()
		{	
			Property<std::string> first_link, last_link;
			char joint_num = 0;
			// clear all buffers
			joint_names_.clear();
			chains_info_.clear();
			joints_index_.clear();
			chains_index_.clear();
			// load new chains	
			this->log(DEBUG) << "Joint groups (kinematics chains):"<< endlog();
			for(int i = 0; i < chains_prop_.size(); i++) {
				// get chain PropertyBag
				Property<PropertyBag> chain_bag = chains_prop_.getItem(i);
				if (!chain_bag.ready()) {
					log(ERROR) << "Incorrect chains description at position " << i << endlog();
					return false;
				}
				// get first and last link names
				first_link = chain_bag.rvalue().getProperty("first_link");
				if (!first_link.ready()) {
					log(ERROR) << "Incorrect first_link property." << endlog();
					return false;
				}
				last_link = chain_bag.rvalue().getProperty("last_link");
				if (!last_link.ready()) {
					log(ERROR) << "Incorrect last_link property." << endlog();
					return false;
				}

				this->log(DEBUG) << " " << chain_bag.getName() << "{ first_link = \"" << first_link.rvalue() << "\", last_link = \"" << last_link.rvalue() << "\" }" << endlog();

				// add chain to index and check dublicates
				if (chains_index_.find(chain_bag.getName()) != chains_index_.end()) {
					this->log(ERROR) << "Dublicate joint group name: " << chain_bag.getName() << endlog();
					return false;
				}
				chains_index_[chain_bag.getName()] = chains_info_.size();

				// construct kinematics chain
				chains_info_.emplace_back();
				ChainInfo& chain_info = chains_info_.back();
				// set name
				chain_info.name = chain_bag.getName();
				// get kdl_chain
				if (!tree_.getChain( first_link, last_link, chain_info.kdl_chain )) {
					this->log(ERROR) << "Unable to construct chain " << chain_bag.getName() << "{ first_link = \"" << first_link.rvalue() << "\", last_link = \"" << last_link.rvalue() << "\" }" << endlog();
					return false;
				}
				// joints numbers
				chain_info.index_begin = joint_names_.size();
				chain_info.size = chain_info.kdl_chain.getNrOfSegments();

				// fill up joints list
				for(int j = 0; j < chain_info.kdl_chain.getNrOfSegments(); j++) {
					const string& name = chain_info.kdl_chain.getSegment(j).getJoint().getName();
					joints_index_[name] = joint_names_.size();
					joint_names_.push_back(name);
				}
			}
			this->log(INFO) << "Loaded " << chains_info_.size() << " chains and " << joint_names_.size() << " joints." <<endlog();
			return true;
		}

		string getRobotDescription() 
		{
			return robot_description_;
		}

		vector<string> listChains()
		{
			vector<string> names;
			for ( const ChainInfo& chain_info : chains_info_ ) {
				names.push_back(chain_info.name);
			}
			return names;
		}

		vector<string> listJoints(const string& name)
		{
			// if chain name is empty return all joints
			if(name == "") return joint_names_;
			// get chain information
			auto it = chains_index_.find(name);
			if (it == chains_index_.end()) return vector<string>(); // not found
			const ChainInfo& chain_info = chains_info_[it->second];
			// return joint list
			return vector<string>(joint_names_.begin() + chain_info.index_begin, joint_names_.begin() + chain_info.index_begin + chain_info.size);
		}

		string getJointChain(const string& name)
		{
			auto it = joints_index_.find(name);
			if (it == joints_index_.end()) return ""; // joint not found!

			int jpos = it->second;
			for(const ChainInfo& chain_info : chains_info_) {
				if ( jpos >= chain_info.index_begin and (jpos < chain_info.index_begin + chain_info.size) ) return chain_info.name; // joint found
			}
			return ""; // joint postion not found!
		}

		vector<string> getJointsChains(const vector<string>& names)
		{
			string chain_name;
			set<string> chain_names;
			for(auto& name : names) {
				chain_name = getJointChain(name);
				if (name != "") {
					chain_names.insert(chain_name);
				}
				this->log(DEBUG) << name << "=" << getJointChain( name ) << endlog();
			}
			return vector<string>(chain_names.begin(), chain_names.end());
		}

		int getJointIndex(const string& name)
		{
			auto iterator = joints_index_.find(name);
			return (iterator == joints_index_.end()) ? -1 : iterator->second;
		}

		Chain * getChain(const string& name)
		{
			auto iterator = chains_index_.find(name);
			if ( iterator == chains_index_.end() ) {
				// limb not found
				return nullptr;
			}
			else { 
				// found
				return & chains_info_[iterator->second].kdl_chain;
			}
		}

		/*Chain getChainImpl(const string& name)
		{
			auto iterator = chains_index_.find(name);
			if ( iterator == chains_index_.end() ) {
				// limb not found
				return Chain();
			}
			else { 
				// found
				return chains_info_[iterator->second].kdl_chain;
			}
		}*/

		bool mapChain(const string& name, sensor_msgs::JointState& joint_state, JntArray& position, JntArray& velocity, JntArray& effort)
		{
			if(!equal( joint_names_.begin(), joint_names_.end(), joint_state.name.begin() )) return false;
			char chain_begin, chain_size;
			const ChainInfo& chain_info = chains_info_[ chains_index_.at(name) ];
			if(joint_state.name.size() == joint_state.position.size() ) {
				// map buffer to JntddArray
				new (&position.data) Eigen::Map<VectorXd>( &joint_state.position[ chain_info.index_begin ], chain_info.size );
			}
			if(joint_state.name.size() == joint_state.velocity.size() ) {
				// map buffer to JntArray
				new (&velocity.data) Eigen::Map<VectorXd>( &joint_state.velocity[ chain_info.index_begin ], chain_info.size );
			}
			if(joint_state.name.size() == joint_state.effort.size() ) {
				// map buffer to JntArray
				new (&effort.data)   Eigen::Map<VectorXd>( &joint_state.effort  [ chain_info.index_begin ], chain_info.size );
			}
			return true;
		}

		bool extractChain(const string& name, const sensor_msgs::JointState& joint_state, JntArray& position, JntArray& velocity, JntArray& effort)
		{
			if(!equal( joint_names_.begin(), joint_names_.end(), joint_state.name.begin() )) return false;
			const ChainInfo& chain_info = chains_info_[ chains_index_.at(name) ];
			if(joint_state.name.size() == joint_state.position.size() ) {
				// copy buffer to JntArray
				position.data = VectorXd::Map( &joint_state.position[ chain_info.index_begin ], chain_info.size );
			}
			if(joint_state.name.size() == joint_state.velocity.size() ) {
				// copy buffer to JntArray
				velocity.data = VectorXd::Map( &joint_state.velocity[ chain_info.index_begin ], chain_info.size );
			}
			if(joint_state.name.size() == joint_state.effort.size() ) {
				// copy buffer to JntArray
				effort.data = VectorXd::Map( &joint_state.effort   [ chain_info.index_begin ], chain_info.size );
			}
			return true;
		}

		bool packChain(const string& name, JntArray& position, JntArray& velocity, JntArray& effort, sensor_msgs::JointState& joint_state)
		{
			Chain chain;
			if(!equal( joint_names_.begin(), joint_names_.end(), joint_state.name.begin() )) return false;

			const ChainInfo& chain_info = chains_info_[ chains_index_.at(name) ];

			if(( position.rows() > 0 ) and ( position.rows() == chain_info.size )) {
				std::copy_n( &position.data[0], chain_info.size, joint_state.position.begin() + chain_info.index_begin );
			}
			if(( velocity.rows() > 0 ) and ( velocity.rows() == chain_info.size )) {
				std::copy_n( &velocity.data[0], chain_info.size, joint_state.velocity.begin() + chain_info.index_begin );
			}
			if(( effort.rows() > 0 ) and ( effort.rows() == chain_info.size )) {
				std::copy_n( &effort.data[0],   chain_info.size, joint_state.effort.begin() + chain_info.index_begin );
			}
			return true;
		}

		string getOwnerName() {
			// getOwner() returns the TaskContext pointer we got in
			// the constructor:
			return getOwner()->getName();
		}
};

} // namespace motion
} // namespace sweetie_bot

ORO_SERVICE_NAMED_PLUGIN(sweetie_bot::motion::RobotModelService, "robot_model")
