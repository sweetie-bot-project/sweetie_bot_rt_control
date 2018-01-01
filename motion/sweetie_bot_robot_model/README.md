# SweetieBot robot model service plugin

This packages contain `robot_model` service which provides access to URDF robot model and semantic robot description (crude analog of MoveIt! SRDF). 
It specifies joint ordering rules in full pose messages, the list of registered kinematic chains (resources), the list of joints in each chain.
Another components use this plugin to extract information about kinematic chains and joints groups which are present in system.

This package is part of [Sweetie Bot project](http://sweetiebot.net). See complete specification [here (Rus)](https://gitlab.com/sweetie-bot/sweetie_doc/wikis/plugin-robotmodel).

Standard usage pattern: load plugin into aggregator component or GlobalService and use ServiceRequester in another component to get access to its operations.

### Properties

1. `robot_description` (`string`) --- robot model in URDF format.
2. `chains` (`PropertyBag`) --- kinematic chains description in following format:
         
        PropertyBag chain_name1 
            string first_link
            string last_link
        PropertyBag chain_name2
		    ...
		...
    
    It can be loaded from cpf file.
     

### Operations

1. `bool configure()` (`OwnThread`) --- parse URDF parameter and extract information about chains.
1. `isConfigured()` (`ClientThread`) --- check if plugin is ready to use. Should be checked before any use.

1. `strings listChains()` (`ClientThread`) --- get list of registered kinematic chains.
1. `vector<string> listJoint(const string& name)` --- return list of joints' names in given kinematic chain. If name is empty strings return full list of known joints.
1. `int getJointIndex(const string& name)` --- returns position of the given joint in full sorted pose.

1. `string getJointChain(string joint)` (`ClientThread`) --- return chain name to which belongs given joint.
1. `strings getJointsChains(strings joints)` (`ClientThread`) --- return chains' names to which belong given joints.

Additional operations may be implemented later:
1. `KDL::Tree getKDLTree()`
1. `KDL::Chain getKDLChain(string name)`
1. `pair<int,int> getChainIndexAndSize(string name)` 
Those operations cannot be used from lua or ops scripts.

Deprecated operations and methods: 
1. `KDL::Chain * getChain(string name)` --- Reason: usage patterns of KDL solvers does not requires fast access to chain in realtime.
2. `extractChain`, `packChain`, `mapChain` --- Reason: it is simpler to use joint induces to extract and pack JntArray to JointState message.

## Usage examples 

See FollowJointState controller.

