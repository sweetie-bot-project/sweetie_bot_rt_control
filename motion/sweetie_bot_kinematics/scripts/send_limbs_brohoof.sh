#!/bin/sh
ROSTOPIC=/kinematics/input_limb_state
ROSTYPE=sweetie_bot_kinematics_msgs/RigidBodyState
VARIABLE="
{
 header: auto,
 name: [leg1, leg2],
 frame:
 [
  {
   p: {x: 0.20531914424, y: 0.038540696666, z: -0.0391293074457},
   M: { data: [-0.15426987564989514, 0.0016131628830781733, -0.9880274303745208, 0.005040128605221191, 0.9999869408391899, 0.0008457273327127924, 0.9880158918613652, -0.004849315064122493, -0.1542759915631728] }
  },
  {
   p: {x: 0.0933447611706, y: -0.0121179089747, z: -0.200714145743},
   M: { data: [0.9999998062047598, -9.387125781603273e-05, 0.0006154499407153042, 9.415349307840062e-05, 0.9999998904240164, -0.0004585706866351569, -0.0006154068266696344, 0.00045862854452810575, 0.9999997054671044] }
  },
 ],
 twist:
 [
  {
   linear: {x: 6.34487176087e-16, y: -6.57800773562e-18, z: -1.38686500422e-15},
   angular: {x: 4.12393330413e-17, y: 1.12505467676e-14, z: -4.77508627843e-17}
  },
  {
   linear: {x: -7.01816787671e-17, y: -2.66821160975e-16, z: -5.09659224919e-17},
   angular: {x: -1.54909044146e-16, y: -1.3697209021e-17, z: -2.97507452432e-18}
  },
 ],
 wrench: []
}"

echo "<<<" $ROSTOPIC "$VARIABLE" "\n<<<"
echo ">>>"
rostopic pub $ROSTOPIC $ROSTYPE --once "$VARIABLE"
echo ">>>"
