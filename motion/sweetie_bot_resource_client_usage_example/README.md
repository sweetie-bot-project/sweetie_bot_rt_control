SWEETIE BOT CONTROLLER SWITCHING INFRASRUCTURE USAGE EXAMPLE
========================================================

### Package content

1. `ControllerTemplate` component is example of an opertion activated controller. 
    Use `start/stop` to activate or deactivate it. `rosdeployment` service can simlify activation from ros worksapce.

    See `scripts/test_resource_control.lua` as deployment example.

2. `ControllerActionlibTemplate` component is example of an actionlib based controller. It implement  
    `sweetie_bot_resource_control_msgs/MoveManually` actionlib server.

    See `scripts/test_resource_control_actionlib.lua` as deployment example.

3. `scripts/resource_control.lua` --- lua module to simplify switching infrastructure deployment.

### Usage

Run roslaunch files `test_controller.launch` and `test_controller_actionlib.launch`. 
`rttlua` working directory has to be set to `<package>/script`. Lua would be able to find
necessary modules othewise. 

### Known issues

1. `ControllerActionlibTemplate` works properly only with OROCOS 2.9  due to `dataOnPortHook()` usage.
