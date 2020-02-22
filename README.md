Sweetie Bot motion control subsystem
====================================

This directory contains component of real-time motion control system of [Sweetie Bot robot](http://sweetie.bot).
Although they are  designed for Sweetie Bot they potentially can be used with any robot. 

Sweetie Bot motion control system is based on [OROCOS real-time framework](http://orocos.org).  OROCOS application consists of a few binary components with well-defined interfaces. 
How these components are connected and interacts with each other is determined by deployment scripts.
This directory contains only OROCOS components.  Deployment scripts are located in `sweetie_bot_deploy` package from [`sweetie_bot`](https://gitlab.com/sweetie-bot/sweetie_bot) repository.

Full description of Sweetie Bot motion control system  concept is located here: [Motion Control Concepts (rus)](https://gitlab.com/sweetie-bot/sweetie_doc/wikis/architecture), [Motion Control Description (rus)](https://gitlab.com/sweetie-bot/sweetie_doc/wikis/motion-control).

#### OROCOS packages:

* `sweetie_bot_logger` --- OROCOS logger extension.
* `motion` --- motion control related components
    * `sweetie_bot_robot_model` --- semantic robot model service. Provides kinematic chains descriptions, contact definitions and so on.
    * `controllers` --- controllers components. They produce reference robot pose depending on current goal. 
    * `sweetie_bot_resource_control` --- resource arbiter prevents conflicts between controllers and pass resources (controlled kinematic chains: legs, head and etc) between them.
    * `sweetie_bot_aggregator` --- aggregate partial reference poses in one full pose.
    * `sweetie_bot_kinematics` --- forward and backward kinematics (kinematic chains level), 
    * `sweetie_bot_odometry` --- calculates base movements given contacts' list.
    * `sweetie_bot_dynamics` --- perform dynamics calculation for current pose: necessary servos efforts, balance conditions, contacts reaction forces.
    * `sweetie_bot_servo_inv` --- translate reference robot pose to servos commands.
* `motion_msgs` --- messages definitions
    * `sweetie_bot_resource_control_msgs` --- internal message for resource management.
    * `sweetie_bot_kinematics_msgs` ---  robot state in convenient for OROCOS subsystem form.
    * `sweetie_bot_control_msgs` ---  control messages, they are used to  control robot from ROS.
* `herkulex` --- hardware driver for Doungbu Herkulex DRS-0101/0201. 
* `sweetie_bot_orocos_misc` --- helper functions.

