#include <sweetie_bot_robot_model/robot_model.hpp>

#include <unordered_map>

#include <rtt/RTT.hpp>
#include <rtt/plugin/ServicePlugin.hpp>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl_typekit/typekit/Types.hpp>

#include <sensor_msgs/typekit/JointState.h>

#include <sweetie_bot_logger/logger.hpp>

//#include <rtt/PropertyBag.hpp>

using namespace std;
using namespace RTT;
using namespace KDL;

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
			string default_contact;
		};
		struct ContactInfo {
			//string name; // contact point name
			std::vector<KDL::Vector> points; // points in the frame of the last segment the kinematic chain.
			//KDL::Jacobian jacobian;  // contact jacobian
		};

	protected:
		// SERVICE INTERFACE
		// parameters
		std::string robot_description_;
		PropertyBag chains_prop_;
		PropertyBag contacts_prop_;
		
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
		// contact points (they can be reordered)
		map<string, ContactInfo> contacts_info_;

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
					 "\t\t\t    PropertyBag chain_name1 { string first_link, string last_link, string default_contact },\n"
					 "\t\t\t    PropertyBag chain_name2 { ... }\n"
					 "\t\t\t    ...\n"
					 "\t\t\t}");
			this->addProperty("contacts", contacts_prop_)
				.doc("Contact description (PropertuBag). 'points' field contains an equivalent set of fixed points in last_link frame. Format: \n"
					 "\t\t\t{\n"
					 "\t\t\t    PropertyBag contact_name1 { KDL::Vector[] points },\n"
					 "\t\t\t    PropertyBag contact_name2 { ... }\n"
					 "\t\t\t    ...\n"
					 "\t\t\t}\n");

			this->addOperation("configure", &RobotModelService::configure, this, OwnThread)
				.doc("Configures service: read parameters, construct kdl tree.");
			this->addOperation("isConfigured", &RobotModelService::isConfigured, this, ClientThread)
				.doc("Return true if service contains a valid robot model.");

			this->addOperation("getRobotDescription", &RobotModelService::getRobotDescription, this, ClientThread)
				.doc("Return robot description string (URDF model).");

			this->addOperation("listChains", &RobotModelService::listChains, this, ClientThread)
				.doc("Return the list of known kinematic chains.");
			this->addOperation("getChainIndex", &RobotModelService::getChainIndex, this, ClientThread)
				.doc("Returns position (index) of the given chain in full pose sorted by chains.")
				.arg("chain", "Chain");
			this->addOperation("getChainDefaultContact", &RobotModelService::getChainDefaultContact, this, ClientThread)
				.doc("Get default contact name for the chain. Return empty string if no default contact supplied.")
				.arg("chain", "Chain name");

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

			this->addOperation("listContacts", &RobotModelService::listContacts, this, ClientThread)
				.doc("Return list of registered contacts.");
			this->addOperation("getContactPoints", &RobotModelService::getContactPoints, this, ClientThread)
				.doc("Return the equivalent fixed points set for the contact. Return empty list if contact not found.")
				.arg("name", "Contact points list. When contact is active they are assumed fixed.");
			this->addOperation("addContactPointsToBuffer", &RobotModelService::addContactPointsToBuffer, this, ClientThread)
				.doc("Return the equivalent fixed points set to buffer. Return number of added points and -1 if contact does not exists.")
				.arg("name", "Contact points list. When contact is active they are assumed fixed.")
				.arg("buffer", "Buffer where points are added.");

			this->addOperation("getKDLChain", &RobotModelService::getKDLChain, this, ClientThread)
				.doc("Get KDL::Chain object. Can form out_of_range exception.");
			this->addOperation("getKDLTree", &RobotModelService::getKDLTree, this, ClientThread)
				.doc("Get KDL::Chain object. Can form out_of_range exception.");
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

			// Get contacts from properties
			if (!readContacts()) {
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
			contacts_info_.clear();
			tree_ = KDL::Tree();
			is_configured = false;
			this->log(INFO) << "RobotModel is cleaned up." <<endlog();
		}

		bool readChains()
		{	
			Property<std::string> first_link, last_link, default_contact;
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
				default_contact = chain_bag.rvalue().getProperty("default_contact");
				if (default_contact.ready() && contacts_info_.find(default_contact.rvalue()) == contacts_info_.end()) {
					log(ERROR) << "Unknown contact name: default_contact = " << default_contact.rvalue() << endlog();
					return false;
				}

				this->log(DEBUG) << chain_bag.getName() << "{ first_link = \"" << first_link.rvalue() << "\", last_link = \"" << last_link.rvalue() << "\" <<\", default_contact = \"" << default_contact.rvalue() << "\" }" << endlog();

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
				// set default contact if present
				if (default_contact.ready()) chain_info.default_contact = default_contact.rvalue();
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

		bool readContacts()
		{	
			// clear all buffers
			contacts_info_.clear();
			// load new chains	
			for(PropertyBag::const_iterator p = contacts_prop_.begin(); p != contacts_prop_.end(); p++) {
				Property<PropertyBag> contact_prop(*p);
				if (!contact_prop.ready()) {
					log(ERROR) << "Incorrect contact property " << contact_prop.getName() << " must be PropertyBag." << endlog();
					return false;
				}
				// get equivalent set of contact points
				Property< std::vector<KDL::Vector> > contact_points_prop = contact_prop.rvalue().getProperty("points");
				if (!contact_points_prop.ready()) { 
					log(ERROR) << "Incorrect contact structure: points field in " << contact_prop.getName() << " must be KDL::Vector[]." << endlog();
					return false;
				}
				// add contact to list
				auto ret = contacts_info_.emplace(contact_prop.getName(), ContactInfo()); 
				if (!ret.second) {
					log(WARN) << "Dublicate contact " << contact_prop.getName() << endlog();
				}
				ret.first->second.points = contact_points_prop.rvalue();

				this->log(DEBUG) << "Contact point " << contact_prop.getName() << " with " <<  ret.first->second.points.size() << " points." << endlog();
			}
			this->log(INFO) << "Loaded " << contacts_info_.size() << " contacts." <<endlog();
			return true;
		}

		const string& getRobotDescription() const
		{
			return robot_description_;
		}

		vector<string> listChains() const
		{
			vector<string> names;
			for ( const ChainInfo& chain_info : chains_info_ ) {
				names.push_back(chain_info.name);
			}
			return names;
		}

		vector<string> listJoints(const string& name) const
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

		string getJointChain(const string& name) const
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

		int getChainIndex(const string& name) const
		{
			auto iterator = chains_index_.find(name);
			return (iterator == chains_index_.end()) ? -1 : iterator->second;
		}

		string getChainDefaultContact(const string& name) const
		{
			auto iterator = chains_index_.find(name);
			return (iterator == chains_index_.end()) ? "" : chains_info_[iterator->second].default_contact;
		}

		int getJointIndex(const string& name) const
		{
			auto iterator = joints_index_.find(name);
			return (iterator == joints_index_.end()) ? -1 : iterator->second;
		}

		Chain getKDLChain(const string& name) const
		{
			auto iterator = chains_index_.find(name);
			if ( iterator == chains_index_.end() ) {
				// limb not found
				return Chain();
			}
			else { 
				//f ound
				return chains_info_[iterator->second].kdl_chain;
			}
		}
		
		Tree getKDLTree() const
		{
			return tree_;
		}

		vector<string> listContacts() const
		{
			vector<string> names;
			for( const auto& pair : contacts_info_ ) names.push_back(pair.first);
			return names;
		}

		vector<KDL::Vector> getContactPoints(const string& name) const
		{
			auto it_pair = contacts_info_.find(name);
			if (it_pair == contacts_info_.end()) return vector<KDL::Vector>();
			else return it_pair->second.points;
		}

		int addContactPointsToBuffer(const string& name, vector<KDL::Vector>& buffer) const
		{
			auto it_pair = contacts_info_.find(name);
			if (it_pair == contacts_info_.end()) return -1;
			else {
				buffer.insert(buffer.end(), it_pair->second.points.begin(), it_pair->second.points.end());
				return it_pair->second.points.size();
			}
		}

};

} // namespace motion
} // namespace sweetie_bot

ORO_SERVICE_NAMED_PLUGIN(sweetie_bot::motion::RobotModelService, "robot_model")
