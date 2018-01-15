#ifndef OROCOS_ROBOT_MODEL_REQUESTER_HPP
#define OROCOS_ROBOT_MODEL_REQUESTER_HPP

#include <rtt/RTT.hpp>
#include <rtt/plugin/PluginLoader.hpp>
//#include <rtt/typekit/Types.hpp>

#include <kdl/chain.hpp>
#include <kdl/tree.hpp>

namespace sweetie_bot {
namespace motion {

class RobotModelInterface
{
   public:
        virtual bool isConfigured() = 0;
        virtual bool configure() = 0;
        virtual void cleanup() = 0;
        virtual const std::string& getRobotDescription() const = 0;
        virtual std::vector<std::string> listChains() const = 0;
        virtual std::vector<std::string> listJoints(const std::string& name) const = 0;
        virtual std::string getJointChain(const std::string& name) const  = 0;
        virtual std::vector<std::string> getJointsChains(const std::vector<std::string>& name) = 0;
        virtual int getChainIndex(const std::string& name) const = 0;
        virtual int getJointIndex(const std::string& name) const = 0;
        virtual KDL::Chain getKDLChain(const std::string& name) const = 0;
        virtual KDL::Tree getKDLTree() const = 0;
};


class RobotModel : public RTT::ServiceRequester {
    public:
        RTT::OperationCaller<bool()> configure;
        RTT::OperationCaller<bool()> isConfigured;
        RTT::OperationCaller<const std::string&()> getRobotDescription;
        RTT::OperationCaller<std::vector<std::string>()> listChains;
        RTT::OperationCaller<std::vector<std::string>(const std::string&)> listJoints;
        RTT::OperationCaller<std::string(const std::string&)> getJointChain;
        RTT::OperationCaller<std::vector<std::string>(const std::vector<std::string>&)> getJointsChains;
        RTT::OperationCaller<int(const std::string&)> getChainIndex;
        RTT::OperationCaller<int(const std::string&)> getJointIndex;
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
            getJointIndex("getJointIndex")
        {
            addOperationCaller(configure);
            addOperationCaller(isConfigured);
            addOperationCaller(getRobotDescription);
            addOperationCaller(listChains);
            addOperationCaller(listJoints);
            addOperationCaller(getJointChain);
            addOperationCaller(getJointsChains);
            addOperationCaller(getChainIndex);
            addOperationCaller(getJointIndex);
        }
};

} // namespace motion
} // namespace sweetie_bot

#endif
