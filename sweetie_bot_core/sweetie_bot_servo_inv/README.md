Packge `sweetie_bot_servo_inv`: servo invertion components
----------------------------------------------------------


This package is part of [Sweetie Bot project](sweetiebot.net). 
Package contains components which translate reference trajectory represented by `sensor_msg::JointState` 
to position controlled servo commands `sweetie_bot_hardware_herkulex_msg::ServoGoal`.

### `PlayerJointState`: simple `JointState` trajectory player

Read trajectory from file and replay it as sequence `sensor_msg::JointState` messages.

File format:

     # commentary
     position[0] velocity[0] position[1] velocity[1] ...
     position[0] velocity[0] position[1] velocity[1] ...
     position[0] velocity[0] position[1] velocity[1] ...
     position[0] velocity[0] position[1] velocity[1] ...

Samples are stored one per line. When component is running each time tick
one sample is published on output port. The tick source can be external 
(`sync` port) or internal (periodic `Activity`).


### `ServoInvLead`: invertion of servo trajectory generator

1. Assume trajectory generator working with rectangular speed profile. (For Herkulex servos set `r{acceletiaon_ratio} = 0`).
2. `ServoGoal` is set to current reference position with lead (usually one or two discretization periods), so servo `r{desired_posion}` 
    register value is equal to reference position with delay. 

#### Usage 

Following script replays trajectory stored in `ref_servos12_sine.in`. 
It relies on `sweetie_bot_hardware_herkulex` infrasructure to transmit commands to servos.

	deployer -s `rospack find sweetie_bot_servo_inv`/scripts/test_servo_inv.ops


