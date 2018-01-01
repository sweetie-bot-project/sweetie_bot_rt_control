#ifndef OROCOS_ROBOT_MODEL_REQUESTER_HPP
#define OROCOS_ROBOT_MODEL_REQUESTER_HPP

#include <rtt/RTT.hpp>
#include <rtt/plugin/PluginLoader.hpp>

#include <sensor_msgs/JointState.h>
#include <kdl/jntarray.hpp>

#include <kdl/chain.hpp>

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
        virtual int getJointIndex(const std::string& name) = 0;
        virtual KDL::Chain * getChain(const std::string& name) = 0; // DEPRECATED
        virtual std::string getOwnerName() = 0; // DEPRECATED
        virtual bool mapChain(const std::string& name, sensor_msgs::JointState& joint_state, KDL::JntArray& position, KDL::JntArray& velocity, KDL::JntArray& effort) = 0;
        virtual bool extractChain(const std::string& name, const sensor_msgs::JointState& joint_state, KDL::JntArray& position, KDL::JntArray& velocity, KDL::JntArray& effort) = 0;
        virtual bool packChain(const std::string& name, KDL::JntArray& position, KDL::JntArray& velocity, KDL::JntArray& effort, sensor_msgs::JointState& joint_state) = 0;
};


class RobotModel : public RTT::ServiceRequester {
    public:
        RTT::OperationCaller<bool()> configure;
        RTT::OperationCaller<std::string()> getRobotDescription;
        RTT::OperationCaller<std::vector<std::string>()> listChains;
        RTT::OperationCaller<std::vector<std::string>(const std::string&)> listJoints;
        RTT::OperationCaller<std::string(const std::string&)> getJointChain;
        RTT::OperationCaller<std::vector<std::string>(const std::vector<std::string>&)> getJointsChains;
        RTT::OperationCaller<int(const std::string&)> getJointIndex;
        RTT::OperationCaller<bool(const std::string&, const sensor_msgs::JointState&, KDL::JntArray&, KDL::JntArray&, KDL::JntArray&)> extractChain;
        RTT::OperationCaller<bool(const std::string&, KDL::JntArray&, KDL::JntArray&, KDL::JntArray&, sensor_msgs::JointState&)> packChain;

        RobotModel(RTT::TaskContext * owner) :
            RTT::ServiceRequester("robot_model", owner),
            configure("configure"),
            getRobotDescription("getRobotDescription"),
            listChains("listChains"),
            listJoints("listJoints"),
            getJointChain("getJointChain"),
            getJointsChains("getJointsChains"),
            getJointIndex("getJointIndex"),
            extractChain("extractChain"),
            packChain("packChain")
        {
            addOperationCaller(configure);
            addOperationCaller(listChains);
            addOperationCaller(listJoints);
            addOperationCaller(getJointChain);
            addOperationCaller(getJointsChains);
            addOperationCaller(getJointIndex);
            addOperationCaller(extractChain);
            addOperationCaller(packChain);
        }
};

} // namespace motion
} // namespace sweetie_bot

#endif
