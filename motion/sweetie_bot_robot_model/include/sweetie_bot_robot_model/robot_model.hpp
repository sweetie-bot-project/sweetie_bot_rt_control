#ifndef OROCOS_ROBOT_MODEL_REQUESTER_HPP
#define OROCOS_ROBOT_MODEL_REQUESTER_HPP

#include <rtt/RTT.hpp>
#include <rtt/plugin/PluginLoader.hpp>
//#include <rtt/typekit/Types.hpp>

#include <kdl/chain.hpp>
#include <kdl/tree.hpp>

#include <kdl_typekit/typekit/Types.hpp>

namespace sweetie_bot {
namespace motion {

class RobotModelInterface
{
   public:
        virtual bool isConfigured() = 0;
        virtual bool configure() = 0;
        virtual void cleanup() = 0;
        virtual std::string getRobotDescription() = 0;
        virtual std::vector<std::string> listChains() = 0;
        virtual std::vector<std::string> listJoints(const std::string& name) = 0;
        virtual std::string getJointChain(const std::string& name) = 0;
        virtual std::vector<std::string> getJointsChains(const std::vector<std::string>& name) = 0;
        virtual int getChainIndex(const std::string& name) const = 0;
        virtual int getJointIndex(const std::string& name) = 0;
		virtual vector<string> listContacts() const = 0;
		virtual vector<KDL::Vector> getContactPoints(const string& name) const = 0;
		virtual int addContactPointsToBuffer(const string& name, vector<KDL::Vector>& buffer) const = 0;
        virtual KDL::Chain getKDLChain(const std::string& name) = 0;
        virtual KDL::Tree getKDLTree() = 0;
};


class RobotModel : public RTT::ServiceRequester {
    public:
        RTT::OperationCaller<bool()> configure;
        RTT::OperationCaller<bool()> isConfigured;
        RTT::OperationCaller<std::string()> getRobotDescription;
        RTT::OperationCaller<std::vector<std::string>()> listChains;
        RTT::OperationCaller<std::vector<std::string>(const std::string&)> listJoints;
        RTT::OperationCaller<std::string(const std::string&)> getJointChain;
        RTT::OperationCaller<std::vector<std::string>(const std::vector<std::string>&)> getJointsChains;
        RTT::OperationCaller<int(const std::string&)> getChainIndex;
        RTT::OperationCaller<int(const std::string&)> getJointIndex;
		RTT::OperationCaller<vector<string> ()> listContacts;
		RTT::OperationCaller<vector<KDL::Vector> (const string&)> getContactPoints;
		RTT::OperationCaller<int (const string&, vector<KDL::Vector>&)> addContactPointsToBuffer;
        RTT::OperationCaller<KDL::Chain(const std::string&)> getKDLChain;
        RTT::OperationCaller<KDL::Tree()> getKDLTree;

        RobotModel(RTT::TaskContext * owner) :
            RTT::ServiceRequester("robot_model", owner),
            configure("configure"),
            isConfigured("isConfigured"),
            getRobotDescription("getRobotDescription"),
            listChains("listChains"),
            listJoints("listJoints"),
            getJointChain("getJointChain"),
            getJointsChains("getJointsChains"),
            getChainIndex("getChainIndex"),
            getJointIndex("getJointIndex"),
			listContacts("listContacts"),
			getContactPoints("getContactPoints"),
			addContactPointsToBuffer("addContactPointsToBuffer"),
            getKDLChain("getKDLChain"),
            getKDLTree("getKDLTree")
        {
            addOperationCaller(configure);
            addOperationCaller(isConfigured);
            addOperationCaller(listChains);
            addOperationCaller(listJoints);
            addOperationCaller(getJointChain);
            addOperationCaller(getJointsChains);
            addOperationCaller(getJointIndex);
            addOperationCaller(getChainIndex);
            addOperationCaller(listContacts);
            addOperationCaller(getContactPoints);
            addOperationCaller(addContactPointsToBuffer);
            addOperationCaller(getKDLChain);
            addOperationCaller(getKDLTree);
        }
};

} // namespace motion
} // namespace sweetie_bot

#endif
