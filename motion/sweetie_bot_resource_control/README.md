Sweetie Bot motion controllers' switching infrastructure 
========================================================

This package contains resource management subsystem. It allows multiply motion controllers to be running concurrently 
without conflicts.  This package is part of [Sweetie Bot project](http://sweetiebot.net). 
See complete specification [here (Rus)](https://gitlab.com/sweetie-bot/sweetie_doc/wikis/components-aggregator-gait).

Resources represents different robot hardware, i.t. `leg1`, `tail`.
When controller is being activated it request necessary resources. ResourcesArbiter forms resource assignment
between controllers and broadcast it. If controller is disagree with resource assignment it can deactivate itself.

Now the most simple strategy resource assignment strategy is implemented: resource goes to the last requester.
So during activation phase controller hijacks resources from other operational controllers. If any of them is unable 
to function without hijacked resources they deactivate itself. Controversially if one controller is deactivated 
resources return to previous owner if it is still active. The same action is taken if controller disowns 
a resource (i.e. the resource is not mentioned in the last resource request).

### Package content

1. `ResourceArbiter` component. It process resource requests from controller and publish resource assignments.

2. `resource_client` service can be loaded into controller and performs interaction with `ResourceArbiter`.
	It provides interface to request arbiter for resources and to process it replies.

	See `sweetie_bot_resource_client_usage_example` package for usage example.

3. `resource_client_dummy` service always believes it owns all necessary resources. It does not interact with 
    `ResourceArbiter`. It can be used for testing purpose.

4. `SimpleControllerBase` superclass which simplifies implementation of SetOperational action activated controllers.


### Usage

See `sweetie_bot_resource_client_usage_example` package.
