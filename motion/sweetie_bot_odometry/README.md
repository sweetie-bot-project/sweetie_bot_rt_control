 Sweetie Bot odometry
========================================

This package provides odometry components.  It is part of [Sweetie Bot project](http://sweetiebot.net). 
See complete specification [here (Rus)](https://gitlab.com/sweetie-bot/sweetie_doc/wikis/components-odometry).

### Components

`Odometry` components calculate base\_link position and velocity provided by information about contacts 
(`in_supports_fixed` port, `SupportState`) and robot limbs positions (`in_limbs_fixed` port, `RigidBodyState`).
Not that order and names of robot limbs must not change when component is running.

If property `force_contact_z_to_zero` is set contact z-coordinate is always assumed equal to zero. 
Current estimate can be overridden by the pose received on `in_base` port.

