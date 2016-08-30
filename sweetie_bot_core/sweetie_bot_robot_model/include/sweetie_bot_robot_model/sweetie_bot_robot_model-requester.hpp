#ifndef OROCOS_ROBOT_MODEL_REQUESTER_HPP
#define OROCOS_ROBOT_MODEL_REQUESTER_HPP

#include <iostream>
#include <memory>

#include <rtt/RTT.hpp>
#include <rtt/plugin/PluginLoader.hpp>

#include <sensor_msgs/JointState.h>
#include <kdl/jntarray.hpp>

#include <kdl/chain.hpp>

using namespace std;
using namespace RTT;
using namespace KDL;

class RobotModelInterface
{
   public:
	/*
	virtual bool isConfigured() = 0;
	virtual bool configure() = 0;
	virtual void cleanup() = 0;
	virtual bool readChains() = 0;
	virtual vector<string> listJoints(const string& name) = 0;
	virtual vector<string> listChains() = 0; */
	virtual Chain * getChain(const string& name) = 0;
	//virtual PropertyBag& getCahinProperties(const string& name) = 0;
	/*
	virtual bool getChainB(const string& name, KDL::Chain& chain) = 0;
	virtual bool mapChain(const string& name, sensor_msgs::JointState& joint_state, JntArray& position, JntArray& velocity, JntArray& effort) = 0;
	virtual bool extractChain(const string& name, const sensor_msgs::JointState& joint_state, JntArray& position, JntArray& velocity, JntArray& effort) = 0;
	virtual bool packChain(const string& name, JntArray& position, JntArray& velocity, JntArray& effort, sensor_msgs::JointState& joint_state) = 0;
	virtual string getOwnerName() = 0; */
};

class RobotModel : public ServiceRequester {
    public:
        OperationCaller<bool()> configure;
        OperationCaller<vector<string>()> listChains;
        OperationCaller<vector<string>(const string&)> listJoints;
        //OperationCaller<bool(const string&, Chain&)> getChainO;
        OperationCaller<bool(const string&, const sensor_msgs::JointState&, JntArray&, JntArray&, JntArray&)> extractChain;
        OperationCaller<bool(const string&, JntArray&, JntArray&, JntArray&, sensor_msgs::JointState&)> packChain;

        RobotModel(TaskContext * owner) :
            ServiceRequester("robot_model_requester", owner),
            configure("configure"),
            listChains("listChains"),
            listJoints("listJoints"),
	    extractChain("extractChain"),
	    packChain("packChain")
        {
            addOperationCaller(configure);
            addOperationCaller(listChains);
            addOperationCaller(listJoints);
            addOperationCaller(extractChain);
            addOperationCaller(packChain);
        }
};

#endif
