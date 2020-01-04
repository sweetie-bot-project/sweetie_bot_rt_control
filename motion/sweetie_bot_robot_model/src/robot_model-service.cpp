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
		struct JointInfo {
			string name; // joint group name
			int group_index; // index of group to which joint belongs
		};
		struct GroupInfo {
			string name; // joint group name
			vector<int> chain_induces; // induces of chains in group
			set<int> joint_induces; // induces of joints in group
		};
		struct ChainInfo {
			string name; // kinematic chain name
			std::vector<int> joint_induces; // induces of joints (with virtual joints)
			int n_real_segments; // chain length (only real segments)
			int n_real_joints; // number of real joints
			int group_index; // index of group to which chain belongs
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
		PropertyBag groups_prop_;
		PropertyBag chains_prop_;
		PropertyBag contacts_prop_;
		
		// SERVICE STATE
		TaskContext* owner_;
		bool is_configured;
		// joints list
		vector<string> joint_names_;
		// joint groups
		vector<GroupInfo> groups_info_; 
		// kinematic chains
		vector<ChainInfo> chains_info_; 
		// contact points
		vector<ContactInfo> contacts_info_;
		// index to speed up search
		map<string, int> joints_index_;
		map<string, int> chains_index_;
		map<string, int> groups_index_;
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
			this->addProperty("groups", groups_prop_)
				.doc("Joints groups descriptions (PropertyBag). Format: \n"
					 "\t\t\t{\n"
					 "\t\t\t    PropertyBag group_name1 { string[] joints, string[] chains },\n"
					 "\t\t\t    PropertyBag group_name2 { ... }\n"
					 "\t\t\t    ...\n"
					 "\t\t\t}");
			this->addProperty("chains", chains_prop_)
				.doc("Kinematics chains descriptions (PropertyBag). Format: \n"
					 "\t\t\t{\n"
					 "\t\t\t    PropertyBag chain_name1 { string first_link, string last_link, string last_link_virtual, string default_contact },\n"
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

			this->addOperation("listGroups", &RobotModelService::listGroups, this, ClientThread)
				.doc("Return the list of known joint groups.");
			this->addOperation("getGroupIndex", &RobotModelService::getGroupIndex, this, ClientThread)
				.doc("Returns position index of the given group in full robot pose vector.")
				.arg("group", "group");
			this->addOperation("getGroupJoints", &RobotModelService::getGroupJoints, this, ClientThread)
				.doc("Return the list of joints in given group.")
				.arg("group", "Group name.");
			this->addOperation("getGroupChains", &RobotModelService::getGroupChains, this, ClientThread)
				.doc("Return the list of chains in given group.")
				.arg("group", "Group name.");
			this->addOperation("getGroupsChains", &RobotModelService::getGroupsChains, this, ClientThread)
				.doc("Return the list of chains in given groups.")
				.arg("groups", "Group list.");

			this->addOperation("listChains", &RobotModelService::listChains, this, ClientThread)
				.doc("Return the list of known kinematic chains.");
			this->addOperation("getChainIndex", &RobotModelService::getChainIndex, this, ClientThread)
				.doc("Returns position (index) of the given chain in full pose.")
				.arg("chain", "Chain");
			this->addOperation("getChainJoints", &RobotModelService::getChainJoints, this, ClientThread)
				.doc("Return the list of joints in given kinematic chain.")
				.arg("chain", "Chain name.");
			this->addOperation("getChainJointsInduces", &RobotModelService::getChainJointsInduces, this, ClientThread)
				.doc("Returns list of induces of the given chain in full pose.")
				.arg("chain", "Chain")
				.arg("with_virtual_joints", "Add virtual joints.");
			this->addOperation("getChainDefaultContact", &RobotModelService::getChainDefaultContact, this, ClientThread)
				.doc("Get default contact name for the chain. Return empty string if no default contact supplied.")
				.arg("chain", "Chain name");
			this->addOperation("getChainProperty", &RobotModelService::getChainProperty, this, ClientThread)
				.doc("Get information about kinematic chain: first_link, last_link, last_link_virtual, default_contact.")
				.arg("chain", "Chain name")
				.arg("property", "Chain property.");
			this->addOperation("getChainGroup", &RobotModelService::getChainGroup, this, ClientThread)
				.doc("Return name of joint group to which belongs given chain.")
				.arg("chain", "Chain name.");
			this->addOperation("getChainsGroups", &RobotModelService::getChainsGroups, this, ClientThread)
				.doc("Returns list of group names to which the given cahin are belongs.")
				.arg("chains", "List of chains names.");

			this->addOperation("listJoints", &RobotModelService::listJoints, this, ClientThread)
				.doc("Return the list of all joints.");
			this->addOperation("getJointIndex", &RobotModelService::getJointIndex, this, ClientThread)
				.doc("Returns position (index) of the given joint in full sorted pose.")
				.arg("joint", "Joint name");
			this->addOperation("getJointGroup", &RobotModelService::getJointGroup, this, ClientThread)
				.doc("Return name of joint group to which belongs given joint.")
				.arg("joint", "Joint name.");
			this->addOperation("getJointsGroups", &RobotModelService::getJointsGroups, this, ClientThread)
				.doc("Returns list of chains names to which the given joints are belongs.")
				.arg("joints", "List of joints names.");

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
				.doc("Get KDL::Tree object. Can form out_of_range exception.");
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

			// Get kinematics chains from properties
			if (!readGroups()) {
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
			groups_info_.clear();
			joints_index_.clear();
			chains_index_.clear();
			groups_index_.clear();
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
			this->log(DEBUG) << "Kinematics chains:"<< endlog();
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
				// get kdl_chain
				if (!tree_.getChain( first_link, last_link_virtual, chain_info.kdl_chain )) {
					this->log(ERROR) << "Unable to construct chain " << chain_bag.getName() << "{ first_link = \"" << first_link.rvalue() << "\", last_link = \"" << last_link_virtual.rvalue() << "\" }" << endlog();
					return false;
				}
				// populate joint list 
				chain_info.n_real_joints = -1;
				for(int j = 0; j < chain_info.kdl_chain.getNrOfSegments(); j++) { // iterate over all jsegments
					const KDL::Segment& segment = chain_info.kdl_chain.getSegment(j);
					// check if corresponding joint is movable. Fixed joints are skipped.
					if (segment.getJoint().getType() != KDL::Joint::None) {
						// segment assotiated with movable joint
						const string& name = segment.getJoint().getName();
						// register joint if it is not registered
						int index;
						auto it = joints_index_.find(name);
						if (it == joints_index_.end()) {
							// new joint, register it 
							index = joint_names_.size();
							joint_names_.push_back(name);
							joints_index_[name] = index;
						}
						else {
							index = it->second;
						}
						// add joint index
						chain_info.joint_induces.push_back(index);
					}
					// real chain size
					if (segment.getName() == last_link.rvalue()) {
						chain_info.n_real_joints = chain_info.joint_induces.size();
						chain_info.n_real_segments = j+1;
					}
				}
				// check if 'last_link' property is correct 
				if (chain_info.n_real_joints < 0) {
					this->log(ERROR) << "Link " << last_link.rvalue() << " is not found between " << first_link.rvalue() << " and " << last_link_virtual.rvalue() << " links" << endlog();
					return false;
				}
				// chain is not assotiated with any group
				chain_info.group_index = -1;
			}
			this->log(INFO) << "Loaded " << chains_info_.size() << " chains and registered " << joint_names_.size() << " joints." <<endlog();
			return true;
		}

		bool readGroups()
		{	
			Property< std::vector<std::string> > chains, joints;
			char joint_num = 0;
			// clear all buffers
			groups_info_.clear();
			groups_index_.clear();
			// load new chains	
			this->log(DEBUG) << "Kinematics groups:"<< endlog();
			for(int i = 0; i < groups_prop_.size(); i++) {
				// get chain PropertyBag
				Property<PropertyBag> group_bag = groups_prop_.getItem(i);
				if (!group_bag.ready()) {
					log(ERROR) << "Incorrect group description at position " << i << endlog();
					return false;
				}
				if (groups_index_.find(group_bag.getName()) != groups_index_.end()) {
					log(ERROR) << "Dublicate group definition " << group_bag.getName() <<  endlog();
					return false;
				}
				// create new group
				groups_index_[group_bag.getName()] = groups_info_.size();
				groups_info_.emplace_back();
				GroupInfo& group_info = groups_info_.back();

				group_info.name = group_bag.getName();
				// process chains
				chains = group_bag.rvalue().getProperty("chains");
				/*if (!chains.ready()) {
					log(ERROR) << "Incorrect chains property." << endlog();
					return false;
				}*/
				if (chains.ready()) {
					for(const std::string& chain_name : chains.rvalue()) {
						auto it = chains_index_.find(chain_name);
						if (it == chains_index_.end()) {
							log(ERROR) << "In group '" << group_bag.getName() << "' unknown chain '" << chain_name << "' is supplied." << endlog();
							return false;
						}
						int chain_index = it->second;
						group_info.chain_induces.push_back(chain_index);
						// add chain joints to the group, 
						group_info.joint_induces.insert(chains_info_[chain_index].joint_induces.begin(), chains_info_[chain_index].joint_induces.end());
						// associate chain with given group
						if (chains_info_[chain_index].group_index != -1) {
							log(ERROR) << "Chain '"  << chain_name << "' belongs to two groups." << endlog();
							return false;
						}
						chains_info_[chain_index].group_index = groups_info_.size() - 1;
					}
				}
				// process joints
				joints = group_bag.rvalue().getProperty("joints");
				/*if (!joints.ready()) {
					log(ERROR) << "Incorrect joints property." << endlog();
					return false;
				}*/
				if (joints.ready()) {
					for(const std::string& joint_name : joints.rvalue()) {
						// check if joint already added
						auto it = joints_index_.find(joint_name);
						if (it != joints_index_.end()) {
							log(ERROR) << "Joint '" << joint_name << "' already belongs to a kinematic chain. Add this chain to group instead of single joint."  << endlog();
						}
						// TODO: maybe it is not necessary: we can add fictive joint which is not in URDF model?
						// check if joint belong to KDL::Tree
						bool joint_found = false;
						for(auto& segment_map_element : tree_.getSegments()) {
							const KDL::Joint& joint = segment_map_element.second.segment.getJoint();
							if (joint.getName() == joint_name && joint.getType() != KDL::Joint::None) {
								joint_found = true;
								break;
							}
						}
						if (!joint_found) {
							log(ERROR) << "Joint '" << joint_name << "' is not defined in URDF model." << endlog();
							return false;
						}
						// add joint
						int joint_index = joint_names_.size();
						joints_index_[joint_name] = joint_index;
						joint_names_.push_back(joint_name);
						// assotiate it with given group
						group_info.joint_induces.insert(joint_index);
					}
				}

				if (log(DEBUG)) {
					log() << "Joint group '" << group_info.name << "' registered with joints: ";
					for (int joint_index : group_info.joint_induces) log() << joint_names_[joint_index] << ", ";
					log() << endlog();
				}
			}
			// check if all chains belongs to groups
			for(const ChainInfo& chain_info : chains_info_) {
				if (chain_info.group_index == -1) {
					log(ERROR) << "Chain '" << chain_info.name << "' is not assigned to any group." << endlog();
				}
			}
			// check conflicts between groups, joints' sets must not intersect
			std::set<int> all_joints;
			for(const GroupInfo& group_info : groups_info_) {
				// check intersection
				for (int index : group_info.joint_induces) {
					if (all_joints.count(index)) {
						log(ERROR) << "Joint '" << joint_names_[index] << "' belongs to multiple groups." << endlog();
						return false;
					}
				}
				all_joints.insert(group_info.joint_induces.begin(), group_info.joint_induces.end());
			}
			this->log(INFO) << "Registered " << groups_info_.size() << " groups. Total number of joints: " << joint_names_.size() << endlog();
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

		// JOINT GROUPS related operations
	
		vector<string> listGroups() const 
		{
			vector<string> names;
			names.reserve(groups_info_.size());
			for ( const GroupInfo& group_info : groups_info_ ) {
				names.push_back(group_info.name);
			}
			return names;
		}

		int getGroupIndex(const string& name) const
		{
			auto iterator = groups_index_.find(name);
			return (iterator == groups_index_.end()) ? -1 : iterator->second;
		}

		vector<string> getGroupJoints(const string& name) const
		{
			// get group information
			auto it = groups_index_.find(name);
			if (it == groups_index_.end()) return vector<string>(); // not found
			const GroupInfo& group_info = groups_info_[it->second];
			// return joint list
			std::vector<string> joints_list;
			joints_list.reserve(group_info.joint_induces.size());
			for(int index : group_info.joint_induces) joints_list.push_back(joint_names_[index]);
			return joints_list;
		}

		vector<string> getGroupChains(const string& name) const
		{
			// get group information
			auto it = groups_index_.find(name);
			if (it == groups_index_.end()) return vector<string>(); // not found
			const GroupInfo& group_info = groups_info_[it->second];
			// return chain list
			std::vector<string> chains_list;
			for(int index : group_info.chain_induces) chains_list.push_back(chains_info_[index].name);
			return chains_list;
		}

		vector<string> getGroupsChains(const vector<string>& names) const
		{
			std::vector<string> chains_list;
			for(const string& name : names) {
				// get group information
				auto it = groups_index_.find(name);
				if (it == groups_index_.end()) continue;
				const GroupInfo& group_info = groups_info_[it->second];
				// populate chains list
				for(int index : group_info.chain_induces) chains_list.push_back(chains_info_[index].name);
			}
			return chains_list;
		}

		// KINEMATIC CHAINS related operations
		
		vector<string> listChains() const
		{
			vector<string> names;
			for ( const ChainInfo& chain_info : chains_info_ ) {
				names.push_back(chain_info.name);
			}
			return names;
		}

		int getChainIndex(const string& name) const
		{
			auto iterator = chains_index_.find(name);
			return (iterator == chains_index_.end()) ? -1 : iterator->second;
		}

		vector<string> getChainJoints(const string& name) const
		{
			// get chain information
			auto it = chains_index_.find(name);
			if (it == chains_index_.end()) return vector<string>(); // not found
			const ChainInfo& chain_info = chains_info_[it->second];
			// return joint list
			std::vector<string> joints_list;
			joints_list.reserve(chain_info.joint_induces.size());
			for(int index : chain_info.joint_induces) joints_list.push_back(joint_names_[index]);
			return joints_list;
		}

		vector<int> getChainJointsInduces(const string& name, bool with_virtual_joints) const
		{
			// TODO return reference 
			// get chain information
			auto it = chains_index_.find(name);
			if (it == chains_index_.end()) return vector<int>(); // not found
			// return joint induces
			const ChainInfo& chain_info = chains_info_[it->second];
			if (with_virtual_joints) return chain_info.joint_induces;
			else return vector<int>(chain_info.joint_induces.begin(), chain_info.joint_induces.begin() + chain_info.n_real_joints);
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
				return chain.kdl_chain.getSegment(chain.n_real_segments-1).getName();
			}
			else if (property == "last_link_virtual") {
				return chain.kdl_chain.getSegment(chain.kdl_chain.getNrOfSegments()-1).getName();
			}
			else if (property == "default_contact") {
				return chain.default_contact;
			}
			else {
				log(ERROR) << "getChainProperty: Unknown property " << property << "." << endlog();
				return "";
			}
		}

		string getChainGroup(const string& name) const
		{
			auto it = chains_index_.find(name);
			if (it != chains_index_.end()) return groups_info_[ chains_info_[it->second].group_index ].name; 
			else return ""; // chain not found!
		}

		vector<string> getChainsGroups(const vector<string>& names)
		{
			string group_name;
			// derive group set
			set<int> group_induces;
			std::vector<string> group_list;
			for(auto& name : names) {
				auto it = chains_index_.find(name);
				if (it != chains_index_.end()) {
					int group_index = chains_info_[it->second].group_index;
					bool new_element;
					std::tie(std::ignore, new_element) = group_induces.insert(group_index);
					if (new_element) group_list.push_back( groups_info_[group_index].name );
				}
			}
			return group_list;
		}

		// JOINTS related operations
		vector<string> listJoints() const
		{
			return joint_names_;
		}

		int getJointIndex(const string& name) const
		{
			auto iterator = joints_index_.find(name);
			return (iterator == joints_index_.end()) ? -1 : iterator->second;
		}

		string getJointGroup(const string& name) const
		{
			auto it = joints_index_.find(name);
			if (it == joints_index_.end()) return ""; // joint not found!

			// find coresponding group
			for(const GroupInfo& group_info : groups_info_) {
				if ( group_info.joint_induces.count(it->second) ) return group_info.name;
			}
			return ""; // joint postion not found!
		}

		vector<string> getJointsGroups(const vector<string>& joint_names)
		{
			// transform joint names to induces
			std::vector<int> joint_induces;
			joint_induces.reserve(joint_names.size());
			for(const string& name : joint_names) joint_induces.push_back( getJointIndex(name) );
			// form corresponding group list
			std::vector<string> groups_names;
			for(const GroupInfo& group_info : groups_info_) {
				if ( std::any_of(joint_induces.begin(), joint_induces.end(), [&group_info](int index) { return group_info.joint_induces.count(index); } ) ) {
					groups_names.push_back( group_info.name );
				}
			}
			return groups_names;
		}

		// KDL MODEL related operations 
	
		Chain getKDLChain(const string& name, bool with_virtual_joints) const
		{
			auto iterator = chains_index_.find(name);
			if ( iterator == chains_index_.end() ) {
				// log(ERROR) << "getKDLChain: Unknown chain " << name << "." << endlog();
				return Chain();
			}

			const ChainInfo& chain_info = chains_info_[iterator->second];
			if (with_virtual_joints || chain_info.n_real_segments == chain_info.kdl_chain.getNrOfSegments()) {
				return chain_info.kdl_chain;
			}
			else {
				KDL::Chain chain;
				for (int i = 0; i < chain_info.n_real_segments; i++) 
					chain.addSegment( chain_info.kdl_chain.getSegment(i) );
				return chain;
			}
		}
		
		Tree getKDLTree() const
		{
			return tree_;
		}

		// Contact related operations

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
