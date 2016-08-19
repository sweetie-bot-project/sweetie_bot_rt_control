#ifndef OROCOS_ROBOT_MODEL_REQUESTER_HPP
#define OROCOS_ROBOT_MODEL_REQUESTER_HPP

#include <iostream>
#include <memory>

#include <rtt/RTT.hpp>
#include <rtt/plugin/PluginLoader.hpp>

#include <sweetie_bot_robot_model/sweetie_bot_robot_model-requester.hpp>

#include <sensor_msgs/JointState.h>
#include <kdl/jntarray.hpp>

#include <kdl/chain.hpp>

using namespace std;
using namespace RTT;
using namespace KDL;

class RobotModel : public ServiceRequester {
    public:
        OperationCaller<bool()> configure;
        OperationCaller<vector<string>()> listChains;
        OperationCaller<vector<string>(const string&)> listJoints;
        OperationCaller<bool(const string&, Chain&)> getChain;
        OperationCaller<bool(const string&, sensor_msgs::JointState&, JntArray&, JntArray&, JntArray&)> mapChain;
        OperationCaller<bool(const string&, const sensor_msgs::JointState&, JntArray&, JntArray&, JntArray&)> copyChain;
        OperationCaller<bool(const string&, JntArray&, JntArray&, JntArray&, sensor_msgs::JointState&)> packChain;

        RobotModel(TaskContext * owner) :
            ServiceRequester("robot_model_requester", owner),
            configure("configure"),
            listChains("listChains"),
            getChain("getChain"),
	    mapChain("mapChain"),
	    copyChain("copyChain"),
	    packChain("packChain")
        {
            addOperationCaller(configure);
            addOperationCaller(listChains);
            addOperationCaller(listJoints);
            addOperationCaller(getChain);
            addOperationCaller(mapChain);
            addOperationCaller(copyChain);
            addOperationCaller(packChain);
        }
};

#endif
