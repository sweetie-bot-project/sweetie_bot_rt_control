#ifndef OROCOS_SWEETIE_BOT_AGREGATOR_COMPONENT_HPP
#define OROCOS_SWEETIE_BOT_AGREGATOR_COMPONENT_HPP

#include <rtt/RTT.hpp>

#include <kdl/chain.hpp>
#include <sweetie_bot_robot_model/sweetie_bot_robot_model-requester.hpp>

using namespace std;

namespace sweetie_bot {

class Agregator : public RTT::TaskContext{

    boost::shared_ptr<RobotModel> robot_model_;
    boost::shared_ptr<RobotModelInterface> robot_model_interface_;

    InputPort<sensor_msgs::JointState> input_port_joint_state_;
    OutputPort<sensor_msgs::JointState> output_port_joint_state_;

    sensor_msgs::JointState input_joint_state_;
    sensor_msgs::JointState output_joint_state_;

    vector<string> chain_names_;
    vector<string> joint_names_;
  public:
    Agregator(string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
};

} // namespace sweetie_bot
#endif
