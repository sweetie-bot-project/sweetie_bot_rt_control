Cartesian space movement controllers
================================

This package contains controllers which are working with targets defined in Cartesian coordinates.
All components implements resource management interface as described in documentation for `sweetie_bot_resource_control` package.
New control cycle of all controllers is triggered by sync port.
It is the part of [Sweetie Bot project](sweetiebot.net).  Full documentation 
is available in Russian [here](https://gitlab.com/sweetie-bot/sweetie_doc/wikis/components-gait).

### Controllers

OROCOS component which control robot movements by publishing reference robot pose.

* `FollowStance` monitor `in_pose_ref` port and move body to desired pose conserving all active contacts.
    If desired pose is unreachable or violate balance conditions movement is performed only partially.

    Algorithm is quite simple. PD regulator is used to translate pose error desired acceleration. 
    Acceleration is passed via `out_base_accel` port inverse dynamics component which calculate admissable acceleration 
    in joint space. Controller receives it on `int_joints_accel_sorted` port and integrate to determine 
    next pose and publish it `out_joints_ref` port.


### Filter plugins

OROCOS services which can be loaded into motion controllers. They are used for trajectory preprocessing.


