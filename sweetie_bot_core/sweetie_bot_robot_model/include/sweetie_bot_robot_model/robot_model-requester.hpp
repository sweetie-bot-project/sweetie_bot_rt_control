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
        virtual bool readChains() = 0;
        virtual std::vector<std::string> listChains() = 0;
        virtual std::vector<std::string> listJoints(const std::string& name) = 0;
        virtual std::string getJointChain(const std::string& name) = 0;
        virtual std::vector<std::string> getJointChains(const std::vector<std::string>& name) = 0;
        virtual int getJointPos(const std::string& name) = 0;
        virtual KDL::Chain * getChain(const std::string& name) = 0;
        virtual bool mapChain(const std::string& name, sensor_msgs::JointState& joint_state, KDL::JntArray& position, KDL::JntArray& velocity, KDL::JntArray& effort) = 0;
        virtual bool extractChain(const std::string& name, const sensor_msgs::JointState& joint_state, KDL::JntArray& position, KDL::JntArray& velocity, KDL::JntArray& effort) = 0;
        virtual bool packChain(const std::string& name, KDL::JntArray& position, KDL::JntArray& velocity, KDL::JntArray& effort, sensor_msgs::JointState& joint_state) = 0;
        virtual std::string getOwnerName() = 0;
};


class RobotModel : public RTT::ServiceRequester {
    public:
        RTT::OperationCaller<bool()> configure;
        RTT::OperationCaller<std::vector<std::string>()> listChains;
        RTT::OperationCaller<std::vector<std::string>(const std::string&)> listJoints;
        RTT::OperationCaller<std::string(const std::string&)> getJointChain;
        RTT::OperationCaller<std::vector<std::string>(const std::vector<std::string>&)> getJointChains;
        RTT::OperationCaller<int(const std::string&)> getJointPos;
        RTT::OperationCaller<bool(const std::string&, const sensor_msgs::JointState&, KDL::JntArray&, KDL::JntArray&, KDL::JntArray&)> extractChain;
        RTT::OperationCaller<bool(const std::string&, KDL::JntArray&, KDL::JntArray&, KDL::JntArray&, sensor_msgs::JointState&)> packChain;

        RobotModel(RTT::TaskContext * owner) :
            RTT::ServiceRequester("robot_model_requester", owner),
            configure("configure"),
            listChains("listChains"),
            listJoints("listJoints"),
            getJointChain("getJointChain"),
            getJointChains("getJointChains"),
            getJointPos("getJointPos"),
            extractChain("extractChain"),
            packChain("packChain")
        {
            addOperationCaller(configure);
            addOperationCaller(listChains);
            addOperationCaller(listJoints);
            addOperationCaller(getJointChain);
            addOperationCaller(getJointChains);
            addOperationCaller(getJointPos);
            addOperationCaller(extractChain);
            addOperationCaller(packChain);
        }
};

} // namespace motion
} // namespace sweetie_bot

#endif
