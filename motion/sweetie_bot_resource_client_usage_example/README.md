SWEETIE BOT CONTROLLER SWITCHING INFRASTRUCTURE USAGE EXAMPLE
========================================================

Sweetie Bot motion controllers examples.
This package is part of [Sweetie Bot project](http://sweetiebot.net). 
See complete specification [here (Rus)](https://gitlab.com/sweetie-bot/sweetie_doc/wikis/components-aggregator-gait).

Package contains following components:

* `ControllerTemplate` component is an example of the operation activated controller. 
    Use `start/stop` to activate or deactivate it. `rosdeployment` service can simlify activation from ros worksapce.

    See `scripts/test_resource_control.lua` as deployment example.

* `ControllerActionlibTemplate` component is an example of the actionlib based controller. It implements
    `sweetie_bot_resource_control_msgs/MoveManually` actionlib server.

    See `scripts/test_resource_control_actionlib.lua` as deployment example.

* Package is missing `SimpleControlerBase` controller template. See @ref `sweetie_bot_resource_control`.
    Use as exmple `FollowPose` or `FollowJointState` controller.

* `scripts/resource_control.lua` --- lua module to simplify switching infrastructure deployment.

### Usage

Run roslaunch files `test_controller.launch` and `test_controller_actionlib.launch`. 
`rttlua` working directory has to be set to `<package>/script`. Lua would be able to find
necessary modules otherwise. 

### Known issues

1. `ControllerActionlibTemplate` works properly only with OROCOS 2.9  due to `dataOnPortHook()` usage.
