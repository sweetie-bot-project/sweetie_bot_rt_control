Misc OROCOS libraries for SweetieBot robot
=====================================

This package is part of [Sweetie Bot project](http://sweetiebot.net). See complete specification [here (Rus)](https://gitlab.com/sweetie-bot/sweetie_doc/wikis/plugin-robotmodel).

* `joint_state_check.hpp` --- functions which can be used to check if`sensor_msgs::JointState` message correctness.
* `mesage_checks.hpp` --- functions to check `JointStateAccel`, `RigidBodyState` and `SupportState` messages
* `get_subservice_by_type.hpp` --- template functions to simplify access to loaded service.
* `simple_action_server.hpp` --- the OROCOS analog of ROS `SimpleActionServer`.
$ `math.hpp` --- quaternions normalization, rotation matrix logarithm calculation.
