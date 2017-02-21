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
	virtual KDL::Chain * getChain(const std::string& name) = 0;
};

class RobotModel : public ServiceRequester {
    public:
        OperationCaller<bool()> configure;
        OperationCaller<std::vector<std::string>()> listChains;
        OperationCaller<std::vector<std::string>(const string&)> listJoints;
        OperationCaller<std::vector<std::string>()> listAllJoints;
        OperationCaller<int(const std::string&)> getJointPos;
        OperationCaller<bool(const std::string&, const sensor_msgs::JointState&, KDL::JntArray&, KDL::JntArray&, KDL::JntArray&)> extractChain;
        OperationCaller<bool(const std::string&, KDL::JntArray&, KDL::JntArray&, KDL::JntArray&, sensor_msgs::JointState&)> packChain;

        RobotModel(TaskContext * owner) :
            ServiceRequester("robot_model_requester", owner),
            configure("configure"),
            listChains("listChains"),
            listJoints("listJoints"),
            listAllJoints("listAllJoints"),
            getJointPos("getJointPos"),
	    extractChain("extractChain"),
	    packChain("packChain")
        {
            addOperationCaller(configure);
            addOperationCaller(listChains);
            addOperationCaller(listJoints);
            addOperationCaller(listAllJoints);
            addOperationCaller(getJointPos);
            addOperationCaller(extractChain);
            addOperationCaller(packChain);
        }
};

} // namespace sweetie_bot

#endif
