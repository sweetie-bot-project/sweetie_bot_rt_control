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

		// joint groups (kinematic chains)
        virtual std::vector<std::string> listChains() const = 0;
        virtual int getChainIndex(const std::string& name) const = 0;
		virtual std::string getChainDefaultContact(const std::string& chain) const = 0;
		virtual std::string getChainProperty(const std::string& name, const std::string& property) = 0;

		// joints
        virtual std::vector<std::string> listJoints(const std::string& name) const = 0;
        virtual int getJointIndex(const std::string& name) const = 0;
        virtual std::string getJointChain(const std::string& name) const = 0;
        virtual std::vector<std::string> getJointsChains(const std::vector<std::string>& name) = 0;

		// contacts
		virtual std::vector<std::string> listContacts() const = 0;
		virtual std::vector<KDL::Vector> getContactPoints(const std::string& name) const = 0;
		virtual int addContactPointsToBuffer(const std::string& name, std::vector<KDL::Vector>& buffer) const = 0;
};


class RobotModel : public RTT::ServiceRequester {
    public:
        RTT::OperationCaller<bool()> configure;
        RTT::OperationCaller<bool()> isConfigured;

        RTT::OperationCaller<const std::string&()> getRobotDescription;
        RTT::OperationCaller<KDL::Chain(const std::string&, bool)> getKDLChain;
        RTT::OperationCaller<KDL::Tree()> getKDLTree;

        RTT::OperationCaller<std::vector<std::string>()> listChains;
        RTT::OperationCaller<int(const std::string&)> getChainIndex;
		RTT::OperationCaller<std::string(const std::string&)> getChainDefaultContact;
		RTT::OperationCaller<std::string(const std::string&, const std::string&)> getChainProperty;

        RTT::OperationCaller<std::vector<std::string>(const std::string&)> listJoints;
        RTT::OperationCaller<int(const std::string&)> getJointIndex;
        RTT::OperationCaller<std::string(const std::string&)> getJointChain;
        RTT::OperationCaller<std::vector<std::string>(const std::vector<std::string>&)> getJointsChains;

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
			// joint groups
            listChains("listChains"),
            getChainIndex("getChainIndex"),
            getChainDefaultContact("getChainDefaultContact"),
			getChainProperty("getChainProperty"),
			// joints
            listJoints("listJoints"),
            getJointIndex("getJointIndex"),
            getJointChain("getJointChain"),
            getJointsChains("getJointsChains"),
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
			// joint groups
            addOperationCaller(listChains);
            addOperationCaller(getChainIndex);
            addOperationCaller(getChainDefaultContact);
			addOperationCaller(getChainProperty);
			// joints
            addOperationCaller(listJoints);
            addOperationCaller(getJointChain);
            addOperationCaller(getJointsChains);
            addOperationCaller(getJointIndex);
			// contacts
            addOperationCaller(listContacts);
            addOperationCaller(getContactPoints);
            addOperationCaller(addContactPointsToBuffer);
        }
};

} // namespace motion
} // namespace sweetie_bot

#endif
