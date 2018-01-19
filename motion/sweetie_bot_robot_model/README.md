# SweetieBot robot model service plugin

This packages contain `robot_model` service which provides access to URDF robot model and semantic robot description (some analog of MoveIt! SRDF). 
It provides to another components information about joint ordering, joints groups (kinematic chains) and possible contacts.
This package is part of [Sweetie Bot project](http://sweetiebot.net). See complete specification [here (Rus)](https://gitlab.com/sweetie-bot/sweetie_doc/wikis/plugin-robotmodel).

**Joint groups**. Joints are grouped in joint groups. Each group is a kinematic chain which contains all joints between first and last link. 
One joint can not be shared between to joint group. Each group has unique index, which specifies its position in full robot pose.

**Joint ordering**. Joints are ordered according to joints groups and then according to their order in kinematic chain which corresponds to th group.

**Contacts**. Contacts are modeled as set of fixed points. Robot model access to these sets. Points coordinates represented in robot segments frames.
Currently contacts are not binded to any specific robot segment, so they can be viewed simply as named points sets.

**Robot description**. Plugin provides access to URDF robot description and KDL models of kinematic chains.

Standard usage pattern: load plugin into aggregator component or GlobalService and then use ServiceRequester in another component to get access to its operations.

### Properties

1. `robot_description` (`string`) --- robot model in URDF format.
2. `chains` (`PropertyBag`) --- kinematic chains description in following format:
         
        PropertyBag chain_name1 
            string first_link
            string last_link
        PropertyBag chain_name2
		    ...
		...
2. `contacts` (`PropertyBag`) --- contacts' description:
         
        PropertyBag contact_name1 
            KDL.Vector[] points
        PropertyBag contact_name2
            KDL.Vector[] points
		    ...
		...
    
Contacts and chains properties can be loaded from .cpf file.
     

### Operations

1. `bool configure()` (`OwnThread`) --- parse URDF parameter and extract information about chains.
1. `isConfigured()` (`ClientThread`) --- check if plugin is ready to use. Should be checked before any use.

1. `strings listChains()` (`ClientThread`) --- get list of registered kinematic chains.
1. `vector<string> listJoint(const string& name)` --- return list of joints' names in given kinematic chain. If name is empty strings return full list of known joints.
1. `int getJointIndex(const string& name)` --- returns position of the given joint in full sorted pose.

1. `string getJointChain(string joint)` (`ClientThread`) --- return chain name to which belongs given joint.
1. `strings getJointsChains(strings joints)` (`ClientThread`) --- return chains' names to which belong given joints.

1. `strings listContacts()` (`ClientThread`) --- return names of all contacts.
1. `KDL.Vector[] getContactPoints(string name)` (`ClientThread`) --- return a set of points equivalent to the contact or nothing if contact does not exists. Points coordinates a given in a link frame.
1. `bool getContactPoints(string name, KDL.Vector[]& buffer)` (`ClientThread`) --- add equivalent contact points to buffer. Return false if contact does no exists.

1. `KDL::Tree getKDLTree()`
1. `KDL::Chain getKDLChain(string name)`

Additional operations may be implemented later:
1. `pair<int,int> getChainIndexAndSize(string name)` 
This operation cannot be used from lua or ops scripts.

## Usage examples 

See FollowJointState controller.

