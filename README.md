Sweetie Bot motion control subsystem
====================================

See [Sweetie Bot project](sweetiebot.net). Full documentation is available in Russian [Motion Control Concepts](https://gitlab.com/sweetie-bot/sweetie_doc/wikis/architecture), [Motion Control Description](https://gitlab.com/sweetie-bot/sweetie_doc/wikis/motion-control).

OROCOS based real-time control system. This folder contains only OROCOS component, 
deployment scripts are located in `sweetie_bot_deploy` package from [`sweetie_bot`](https://gitlab.com/sweetie-bot/sweetie_bot) repository.

OROCOS packages:

* `sweetie_bot_logger` --- OROCOS logger extension.
* `motion` --- motion control related coponents
    * `sweetie_bot_robot_model` --- semantic robot model service. Provides kinematic chains descriptions, contact definitions and so on.
    * `controllers` --- controllers components. They produce reference robot pose depending on current goal. The pose may be partial.
    * `sweetie_bot_resource_control` --- resource arbiter prevents conflicts between controllers and pass resources (controlled kinematic chains: legs, head and etc) between them.
    * `sweetie_bot_aggregator` --- aggregate partial reference poses in one full pose.
    * `sweetie_bot_kinematics` --- forward and backward kinematics (kinematic chains level), 
    * `sweetie_bot_odometry` --- calculates base movements given contacts' list.
    * `sweetie_bot_dynamics` --- perform dynamics calculation for current pose: necessary servos efforts, balance conditions, contacts reaction forces.
    * `sweetie_bot_servo_inv` --- translate reference robot pose to servos commands.
* `herkulex` --- Herkulex DRS-0101/0201 interface components. Pass servos' commands to the servos.
* `sweetie_bot_orocos_misc` --- helper functions.


