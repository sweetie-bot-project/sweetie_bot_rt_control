# SweetieBot robot model service plugin
This package is part of [Sweetie Bot project](http://sweetiebot.net). See complete specification [here (Rus)](https://gitlab.com/sweetie-bot/sweetie_doc/wikis/plugin-robotmodel).

Robot model plugin contains service that can be loaded to the other OROCOS packages and provide access to the kinematic model and chains enumeration functions.

### Operations

1. `configure()`/`cleanup()` (`OwnThread`) --- urdf loading, construct kdl tree, chains creation / cleanup.
1. `strings listChains()` (`ClientThread`) --- Lists loaded chains.
1. `vector<string> listJoints(const string& name)` --- Lists joints in chain.
1. `vector<string> listAllJoints()` --- Lists all joints in all chains.
1. `int getJointPos(const string& name)` --- Returns position of the given joint name in sorted pose.
1. `bool extractChain(const string& name, const sensor_msgs::JointState& joint_state, JntArray& position, JntArray& velocity, JntArray& effort)` (`ClientThread`)
    --- extracting of given kinematic chain.
1. `bool packChain(const string& name, JntArray& position, JntArray& speed, JntArray& efforts, JointState& in)` (`ClientThread`) 
     --- retroactivity, Returns true if successful (false: chain absent, component is not ready).

### Fields

1. `chains` (`map<string,KDL::Chain *>`) --- kinematic chains.

### Methods

1. `KDL::Chain& getCahin(const string& name)`  --- access to the chains.
1. `PropertyBag& getCahinProperties(const string& name)` --- easier access to additional chains parameters.

## How to use

#### 1. Add this package as a dependent to `CMakeLists.txt`
```
orocos_generate_package(DEPENDS sweetie_bot_robot_model)
```

#### 2. And to the `package.xml`
```
<build_depend>sweetie_bot_robot_model</build_depend>
<run_depend>sweetie_bot_robot_model</run_depend>
```
#### 3. Include requester
```
#include <sweetie_bot_robot_model/sweetie_bot_robot_model-requester.hpp>
```
#### 4. Declare RobotModel class variables
```
boost::shared_ptr<RobotModel> robot_model_;
boost::shared_ptr<RobotModelInterface> robot_model_interface_;
```
#### 5. Initialize them (in a component constrictor)
```
  robot_model_ = getProvider<RobotModel>("robot_model"); // It tries to load the service if it is not loaded.
  robot_model_interface_ = boost::dynamic_pointer_cast<RobotModelInterface>(this->provides()->getService("robot_model"));
```
#### 6. Check if they loaded propperly (in configureHook of component)
```
  if((nullptr == robot_model_) or (nullptr == robot_model_interface_)) return false;
```
#### 7. Configure robot_model
This function initializes plugin, it reads `robot_description` property, loads `URDF` robot model and constructed kdl tree (property must be filled before that).
```
  if(!robot_model_->configure()) return false;
```
#### 8. Use it
```
  vector<string> chain_names = robot_model_->listChains();
  vector<string> joints = robot_model_->listJoints(name);
  vector<string> joints = robot_model_->listAllJoints();
  Chain * chain = robot_model_interface_->getChain(name);
  robot_model_->extractChain(name, input_joint_seed_, position, velocity, effort);
  robot_model_->packChain( name, return_joints_positions, zero_joints_speed, zero_joints_effort, output_joint_state_);
```
