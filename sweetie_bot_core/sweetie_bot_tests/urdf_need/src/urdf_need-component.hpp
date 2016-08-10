#ifndef OROCOS_URDF_NEED_COMPONENT_HPP
#define OROCOS_URDF_NEED_COMPONENT_HPP

#include <iostream>
#include <memory>

#include <rtt/RTT.hpp>
#include <rtt/plugin/PluginLoader.hpp>

#include <sensor_msgs/JointState.h>
#include <kdl/jntarray.hpp>

using namespace std;
using namespace RTT;

using namespace KDL;

class RobotModel : public ServiceRequester { // внешний интерфейс для удаленного вызова операций сервиса (опционален)
    public:
        OperationCaller<bool()> configure;
        OperationCaller<vector<string>()> listChains;
        OperationCaller<bool(const string&, const sensor_msgs::JointState&, JntArray&, JntArray&, JntArray&)> extractChain;
        OperationCaller<bool(const string&, JntArray&, JntArray&, JntArray&, sensor_msgs::JointState&)> packChain;

        RobotModel(TaskContext * owner) :
            ServiceRequester("robot_model_requester", owner),
            configure("configure"),
            listChains("listChains"),
	    extractChain("extractChain"),
	    packChain("packChain")
        {
            addOperationCaller(configure);
            addOperationCaller(listChains);
            addOperationCaller(extractChain);
            addOperationCaller(packChain);
        }
};


class Urdf_need : public RTT::TaskContext{
    boost::shared_ptr<RobotModel> robot_model_req;
    InputPort<sensor_msgs::JointState> inJointState;
  public:
    Urdf_need(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
};
#endif
