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
        virtual KDL::Chain getKDLChain(const std::string& name) const = 0;
        virtual KDL::Tree getKDLTree() const = 0;

		// joint groups (kinematic chains)
        virtual std::vector<std::string> listChains() const = 0;
        virtual int getChainIndex(const std::string& name) const = 0;

		// joints
        virtual std::vector<std::string> listJoints(const std::string& name) const = 0;
        virtual int getJointIndex(const std::string& name) const = 0;
        virtual std::string getJointChain(const std::string& name) const = 0;
        virtual std::vector<std::string> getJointsChains(const std::vector<std::string>& name) = 0;

		// contacts
		virtual std::vector<std::string> listContacts() const = 0;
		virtual vector<KDL::Vector> getContactPoints(const std::string& name) const = 0;
		virtual int addContactPointsToBuffer(const string& name, vector<KDL::Vector>& buffer) const = 0;
};


class RobotModel : public RTT::ServiceRequester {
    public:
        RTT::OperationCaller<bool()> configure;
        RTT::OperationCaller<bool()> isConfigured;

        RTT::OperationCaller<std::string()> getRobotDescription;
        RTT::OperationCaller<KDL::Chain(const std::string&)> getKDLChain;
        RTT::OperationCaller<KDL::Tree()> getKDLTree;

        RTT::OperationCaller<std::vector<std::string>()> listChains;
        RTT::OperationCaller<int(const std::string&)> getChainIndex;

        RTT::OperationCaller<std::vector<std::string>(const std::string&)> listJoints;
        RTT::OperationCaller<int(const std::string&)> getJointIndex;
        RTT::OperationCaller<std::string(const std::string&)> getJointChain;
        RTT::OperationCaller<std::vector<std::string>(const std::vector<std::string>&)> getJointsChains;

		RTT::OperationCaller<vector<string> ()> listContacts;
		RTT::OperationCaller<vector<KDL::Vector> (const string&)> getContactPoints;
		RTT::OperationCaller<int (const string&, vector<KDL::Vector>&)> addContactPointsToBuffer;

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
			// joints
            listJoints("listJoints"),
            getJointChain("getJointChain"),
            getJointsChains("getJointsChains"),
            getJointIndex("getJointIndex"),
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
