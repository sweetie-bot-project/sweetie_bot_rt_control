 #ifndef OROCOS_ROBOT_MODEL_REQUESTER_HPP
#define OROCOS_ROBOT_MODEL_REQUESTER_HPP

#include <rtt/RTT.hpp>

#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl_typekit/typekit/Types.hpp>

namespace sweetie_bot {
namespace motion {

class RobotModelInterface
{
   public:
		// configuartion	   
        virtual bool isConfigured() = 0;
        virtual bool configure() = 0;
        virtual void cleanup() = 0;

		// robot description
        virtual const std::string& getRobotDescription() const = 0;
        virtual KDL::Chain getKDLChain(const std::string& name, bool with_virtual_joints) const = 0;
        virtual KDL::Tree getKDLTree() const = 0;

		// kinematic chains
        virtual std::vector<std::string> listChains() const = 0;
        virtual int getChainIndex(const std::string& chain) const = 0;
        virtual std::vector<std::string> getChainJoints(const std::string& chain) const = 0;
        virtual std::vector<int> getChainJointsInduces(const std::string& chain, bool with_virtual_joints) const = 0;
		virtual std::string getChainDefaultContact(const std::string& chain) const = 0;
		virtual std::string getChainProperty(const std::string& name, const std::string& property) = 0;
        virtual std::string getChainGroup(const std::string& chain) const = 0;
        virtual std::vector<std::string> getChainsGroups(const std::vector<std::string>& chains) = 0;

		// joint groups
        virtual std::vector<std::string> listGroups() const = 0;
        virtual int getGroupIndex(const std::string& group) const = 0;
        virtual std::vector<std::string> getGroupJoints(const std::string& group) const = 0;
        virtual std::vector<std::string> getGroupChains(const std::string& group) const = 0;
        virtual std::vector<std::string> getGroupsChains(const std::vector<std::string>& groups) const = 0;

		// joints
        virtual std::vector<std::string> listJoints() const = 0;
        virtual int getJointIndex(const std::string& joint) const = 0;
        virtual std::string getJointGroup(const std::string& joint) const = 0;
        virtual std::vector<std::string> getJointsGroups(const std::vector<std::string>& joints) = 0;

		// contacts
		virtual std::vector<std::string> listContacts() const = 0;
		virtual std::vector<KDL::Vector> getContactPoints(const std::string& name) const = 0;
		virtual int addContactPointsToBuffer(const std::string& name, std::vector<KDL::Vector>& buffer) const = 0;
};

class RobotModel : public RTT::ServiceRequester {
    public:
		// configuration
        RTT::OperationCaller<bool()> configure;
        RTT::OperationCaller<bool()> isConfigured;

		// robot description
        RTT::OperationCaller<const std::string&()> getRobotDescription;
        RTT::OperationCaller<KDL::Chain(const std::string&, bool)> getKDLChain;
        RTT::OperationCaller<KDL::Tree()> getKDLTree;

		// kinematic chains
        RTT::OperationCaller<std::vector<std::string>()> listChains;
        RTT::OperationCaller<int(const std::string&)> getChainIndex;
        RTT::OperationCaller<std::vector<std::string>(const std::string&)> getChainJoints;
        RTT::OperationCaller<std::vector<int>(const std::string&, bool)> getChainJointsInduces;
		RTT::OperationCaller<std::string(const std::string&)> getChainDefaultContact;
		RTT::OperationCaller<std::string(const std::string&, const std::string&)> getChainProperty;
        RTT::OperationCaller<std::string(const std::string&)> getChainGroup;
        RTT::OperationCaller<std::vector<std::string>(const std::vector<std::string>&)> getChainsGroups;

		// joint groups
        RTT::OperationCaller<std::vector<std::string>()> listGroups;
        RTT::OperationCaller<int(const std::string&)> getGroupIndex;
        RTT::OperationCaller<std::vector<std::string>(const std::string&)> getGroupJoints;
        RTT::OperationCaller<std::vector<std::string>(const std::string&)> getGroupChains;
        RTT::OperationCaller<std::vector<std::string>(const std::vector<std::string>&)> getGroupsChains;

		// joints
        RTT::OperationCaller<std::vector<std::string>()> listJoints;
        RTT::OperationCaller<int(const std::string&)> getJointIndex;
        RTT::OperationCaller<std::string(const std::string&)> getJointGroup;
        RTT::OperationCaller<std::vector<std::string>(const std::vector<std::string>&)> getJointsGroups;

		// contacts
		RTT::OperationCaller<std::vector<std::string> ()> listContacts;
		RTT::OperationCaller<std::vector<KDL::Vector> (const std::string&)> getContactPoints;
		RTT::OperationCaller<int (const std::string&, std::vector<KDL::Vector>&)> addContactPointsToBuffer;

        RobotModel(RTT::TaskContext * owner) :
            RTT::ServiceRequester("robot_model", owner),
			// configuration 
            configure("configure"),
            isConfigured("isConfigured"),
			// robot description
            getRobotDescription("getRobotDescription"),
            getKDLChain("getKDLChain"),
            getKDLTree("getKDLTree"),
			// kinematic chains
            listChains("listChains"),
            getChainIndex("getChainIndex"),
			getChainJoints("getChainJoints"),
			getChainJointsInduces("getChainJointsInduces"),
            getChainDefaultContact("getChainDefaultContact"),
			getChainProperty("getChainProperty"),
			getChainGroup("getChainGroup"),
			getChainsGroups("getChainsGroups"),
			// joint groups
            listGroups("listGroups"),
            getGroupIndex("getGroupIndex"),
			getGroupJoints("getGroupJoints"),
			getGroupChains("getGroupChains"),
			getGroupsChains("getGroupsChains"),
			// joints
            listJoints("listJoints"),
            getJointIndex("getJointIndex"),
            getJointGroup("getJointGroup"),
            getJointsGroups("getJointsGroups"),
			// contacts 
			listContacts("listContacts"),
			getContactPoints("getContactPoints"),
			addContactPointsToBuffer("addContactPointsToBuffer")
        {
            addOperationCaller(configure);
            addOperationCaller(isConfigured);
			// robot description
			addOperationCaller(getRobotDescription);
            addOperationCaller(getKDLChain);
            addOperationCaller(getKDLTree);
			// kinematic chains
            addOperationCaller(listChains);
            addOperationCaller(getChainIndex);
			addOperationCaller(getChainJoints),
			addOperationCaller(getChainJointsInduces),
            addOperationCaller(getChainDefaultContact);
			addOperationCaller(getChainProperty);
            addOperationCaller(getChainGroup);
            addOperationCaller(getChainsGroups);
			// joint groups
            addOperationCaller(listGroups);
            addOperationCaller(getGroupIndex);
            addOperationCaller(getGroupJoints);
            addOperationCaller(getGroupChains);
            addOperationCaller(getGroupsChains);
			// joints
            addOperationCaller(listJoints);
            addOperationCaller(getJointIndex);
            addOperationCaller(getJointGroup);
            addOperationCaller(getJointsGroups);
			// contacts
            addOperationCaller(listContacts);
            addOperationCaller(getContactPoints);
            addOperationCaller(addContactPointsToBuffer);
        }
};

} // namespace motion
} // namespace sweetie_bot

#endif
