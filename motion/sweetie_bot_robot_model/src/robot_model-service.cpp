#include <sweetie_bot_robot_model/robot_model.hpp>

#include <unordered_map>

#include <rtt/RTT.hpp>
#include <rtt/plugin/ServicePlugin.hpp>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl_typekit/typekit/Types.hpp>

#include <sensor_msgs/typekit/JointState.h>

#include <sweetie_bot_logger/logger.hpp>

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
			int size; // chain length (only real joints)
			int size_virtual; // chain length including fictive joints
			KDL::Chain kdl_chain; // KDL chain (with virtual joints)
			string default_contact;
		};
		struct ContactInfo {
			string name; // contact point name
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
		// contact points
		vector<ContactInfo> contacts_info_;
		// index to speed up search
		map<string, int> joints_index_;
		map<string, int> chains_index_;
		map<string, int> contacts_index_;

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
				.doc("Joint groups (kinematics chains) descriptions (PropertyBag). Format: \n"
					 "\t\t\t{\n"
					 "\t\t\t    PropertyBag chain_name1 { string first_link, string last_link, string default_contact },\n"
					 "\t\t\t    PropertyBag chain_name2 { ... }\n"
					 "\t\t\t    ...\n"
					 "\t\t\t}");
			this->addProperty("contacts", contacts_prop_)
				.doc("Contact description (PropertyBag). 'points' field contains an equivalent set of fixed points in last_link frame. Format: \n"
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
			this->addOperation("getChainProperty", &RobotModelService::getChainProperty, this, ClientThread)
				.doc("Get information about kinematic chain: first_link, last_link, last_link_virtual, default_contact.")
				.arg("chain", "Chain name")
				.arg("property", "Chain property.");

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
				.doc("Get KDL::Chain object.")
				.arg("chain", "Chain name")
				.arg("with_virtual_joints", "Add to chain virtual joints.");
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
			// Check cpf file is loaded
			if(chains_prop_.empty())
			{
				log(ERROR) << "The property 'chains' is empty! Check .cpf file existence and its loaded correctly." <<endlog();
				return false;
			}

			if (is_configured) {
				this->log(WARN) << "RobotModel is already configured!" << endlog();
				return true;
			}

			// Load URDF robot description into KDL Tree
			if( robot_description_ == "")
			{
				log(ERROR) << "robot_description parameter is empty!" <<endlog();
				return false;
			}

			if (!kdl_parser::treeFromString(robot_description_, tree_)) {
				this->log(ERROR) << "Failed to construct kdl tree!" <<endlog();
				cleanup();
				return false;
			}
			log(DEBUG) << "Loaded " << tree_.getNrOfSegments() << " segments and " << tree_.getNrOfJoints() << " joints from robot_description to KDL::Tree" <<endlog();

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
			contacts_index_.clear();
			tree_ = KDL::Tree();
			is_configured = false;
			this->log(INFO) << "RobotModel is cleaned up." <<endlog();
		}

		bool readChains()
		{	
			Property<std::string> first_link, last_link, last_link_virtual, default_contact;
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
				last_link_virtual = chain_bag.rvalue().getProperty("last_link_virtual");
				if (!last_link_virtual.ready()) last_link_virtual = last_link;

				default_contact = chain_bag.rvalue().getProperty("default_contact");
				if (default_contact.ready() && contacts_index_.find(default_contact.rvalue()) == contacts_index_.end()) {
					log(ERROR) << "Unknown contact name: default_contact = " << default_contact.rvalue() << endlog();
					return false;
				}

				if (log(DEBUG)) {
				   log() << chain_bag.getName() << "{ first_link = \"" << first_link.rvalue() << "\", last_link = \"" << last_link.rvalue() << "\", last_link_virtual = \"" << last_link_virtual.rvalue() << "\"";
				   if (default_contact.ready()) log() << ", default_contact = \"" << default_contact.rvalue() << "\"";
				   log() << " }" << endlog();
				}

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
				// get kdl_chains
				if (!tree_.getChain( first_link, last_link_virtual, chain_info.kdl_chain )) {
					this->log(ERROR) << "Unable to construct chain " << chain_bag.getName() << "{ first_link = \"" << first_link.rvalue() << "\", last_link = \"" << last_link_virtual.rvalue() << "\" }" << endlog();
					return false;
				}
				// joints numbers
				chain_info.index_begin = joint_names_.size();
				chain_info.size_virtual = chain_info.kdl_chain.getNrOfSegments();

				// fill up joints list
				chain_info.size = -1; // size undefined
				for(int j = 0; j < chain_info.kdl_chain.getNrOfSegments(); j++) { // iterate over all
					const KDL::Segment& segment = chain_info.kdl_chain.getSegment(j);
					const string& name = segment.getJoint().getName();
					if (segment.getJoint().getType() == KDL::Joint::None) {
						this->log(ERROR) << "Joints of type 'None' is not supported. Joint:  " << name << endlog();
						return false;
					}
					joints_index_[name] = joint_names_.size();
					joint_names_.push_back(name);

					if (segment.getName() == last_link.rvalue()) chain_info.size = j+1;
				}
				if (chain_info.size < 0) {
					this->log(ERROR) << "Link " << last_link.rvalue() << " is not found between " << first_link.rvalue() << " and " << last_link_virtual.rvalue() << " links" << endlog();
					return false;
				}
			}
			this->log(INFO) << "Loaded " << chains_info_.size() << " chains and " << joint_names_.size() << " joints." <<endlog();
			return true;
		}

		bool readContacts()
		{	
			// clear all buffers
			contacts_info_.clear();
			contacts_index_.clear();
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
				
				auto ret = contacts_index_.insert( std::make_pair( contact_prop.getName(), contacts_info_.size()) );
				// check for dublicates
				if (!ret.second) {
					log(WARN) << "Dublicate contact " << contact_prop.getName() << ". Skip point." << endlog();
					continue;
				}
				// add contact to list
				contacts_info_.emplace_back(); 
				ContactInfo& contact = contacts_info_.back();
				contact.name = contact_prop.getName();
				contact.points = contact_points_prop.rvalue();

				this->log(DEBUG) << "Contact point " << contact.name << " with " <<  contact.points.size() << " points." << endlog();
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
			return vector<string>(joint_names_.begin() + chain_info.index_begin, joint_names_.begin() + chain_info.index_begin + chain_info.size_virtual);
		}

		string getJointChain(const string& name) const
		{
			auto it = joints_index_.find(name);
			if (it == joints_index_.end()) return ""; // joint not found!

			int jpos = it->second;
			for(const ChainInfo& chain_info : chains_info_) {
				if ( jpos >= chain_info.index_begin and (jpos < chain_info.index_begin + chain_info.size_virtual) ) return chain_info.name; // joint found
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

		string getChainProperty(const string& name, const string& property)
		{
			auto iterator = chains_index_.find(name);
			if (iterator == chains_index_.end()) return "";
			const ChainInfo& chain = chains_info_[iterator->second];
		   	// get parameter
			if (property == "first_link") {
				KDL::SegmentMap::const_iterator element = tree_.getSegment(chain.kdl_chain.getSegment(0).getName());
				element = element->second.parent;
				return element->second.segment.getName();
			} 
			else if (property == "last_link") {
				return chain.kdl_chain.getSegment(chain.size-1).getName();
			}
			else if (property == "last_link_virtual") {
				return chain.kdl_chain.getSegment(chain.size_virtual-1).getName();
			}
			else if (property == "default_contact") {
				return chain.default_contact;
			}
			else {
				log(ERROR) << "getChainProperty: Unknown property " << property << "." << endlog();
				return "";
			}
		}

		int getJointIndex(const string& name) const
		{
			auto iterator = joints_index_.find(name);
			return (iterator == joints_index_.end()) ? -1 : iterator->second;
		}

		Chain getKDLChain(const string& name, bool with_virtual_joints) const
		{
			auto iterator = chains_index_.find(name);
			if ( iterator == chains_index_.end() ) {
				// log(ERROR) << "getKDLChain: Unknown chain " << name << "." << endlog();
				return Chain();
			}

			const ChainInfo& chain_info = chains_info_[iterator->second];
			if (with_virtual_joints || chain_info.size == chain_info.size_virtual ) {
				return chain_info.kdl_chain;
			}
			else {
				KDL::Chain chain;
				for (int i = 0; i < chain_info.size; i++) chain.addSegment(chain_info.kdl_chain.getSegment(i));
				return chain;
			}
		}
		
		Tree getKDLTree() const
		{
			return tree_;
		}

		vector<string> listContacts() const
		{
			vector<string> names;
			for( const ContactInfo& contact : contacts_info_ ) names.push_back(contact.name);
			return names;
		}

		vector<KDL::Vector> getContactPoints(const string& name) const
		{
			auto it = contacts_index_.find(name);
			if (it == contacts_index_.end()) return vector<KDL::Vector>();
			else return contacts_info_[it->second].points;
		}

		int addContactPointsToBuffer(const string& name, vector<KDL::Vector>& buffer) const
		{
			auto it = contacts_index_.find(name);
			if (it == contacts_index_.end()) return -1;
			else {
				buffer.insert( buffer.end(), contacts_info_[it->second].points.begin(), contacts_info_[it->second].points.end() );
				return contacts_info_[it->second].points.size();
			}
		}

};

} // namespace motion
} // namespace sweetie_bot

ORO_SERVICE_NAMED_PLUGIN(sweetie_bot::motion::RobotModelService, "robot_model")
