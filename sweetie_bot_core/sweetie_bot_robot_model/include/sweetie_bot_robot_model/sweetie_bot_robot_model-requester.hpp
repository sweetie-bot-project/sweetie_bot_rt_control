#ifndef OROCOS_ROBOT_MODEL_REQUESTER_HPP
#define OROCOS_ROBOT_MODEL_REQUESTER_HPP

#include <rtt/RTT.hpp>
#include <rtt/plugin/PluginLoader.hpp>

#include <sensor_msgs/JointState.h>
#include <kdl/jntarray.hpp>

#include <kdl/chain.hpp>

using namespace std;
using namespace RTT;
using namespace KDL;

namespace sweetie_bot {

class RobotModelInterface
{
   public:
	virtual Chain * getChain(const string& name) = 0;
};

class RobotModel : public ServiceRequester {
    public:
        OperationCaller<bool()> configure;
        OperationCaller<vector<string>()> listChains;
        OperationCaller<vector<string>(const string&)> listJoints;
        OperationCaller<vector<string>()> listAllJoints;
        OperationCaller<int(const string&)> getJointPos;
        OperationCaller<bool(const string&, const sensor_msgs::JointState&, JntArray&, JntArray&, JntArray&)> extractChain;
        OperationCaller<bool(const string&, JntArray&, JntArray&, JntArray&, sensor_msgs::JointState&)> packChain;

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
