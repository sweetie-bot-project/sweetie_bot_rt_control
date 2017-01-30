SWEETIE BOT CONTROLLER SWITCHING INFRASRUCTURE 
==============================================

This infrastructure allows motions controllers to avoid resource conflicts when they are lauched concurrenly.
Resources are represented by strings which coresponds to hardware parts of robot, i.t. `leg1`, `tail`.
When controller is being activated it request necessary resources. ResourcesArbiter forms resource assigment
between controllers and broadcast it. If controller is disagree with resource assigment it can deactivate itself.

Now implemented the most simple srategy: resource goes to the last requster. So basically during activation
controller hijacks resources from operational controllers. If any of them is unable to function without 
hijacked resources they deactivate itself.


### Package content

1. `ResourceArbiter` component. It process resource requests from controller and publish resource assigments.

2. `resource_client` service can be loaded into controller and performs interaction with `ResourceArbiter`.
	See deatiled description below. (TODO)

3. `resource_client_dummy` service always belives it own all necessary resources. It does not intearct with 
    `ResourceArbiter`. It can be used for testing purpose.

4. `OrocosSimpleActionServer` class simplifies implementation of actionlib based components. 
    See detailed description below. (TODO)

### Usage

See `sweetie_bot_resource_client_usage_example` package.
