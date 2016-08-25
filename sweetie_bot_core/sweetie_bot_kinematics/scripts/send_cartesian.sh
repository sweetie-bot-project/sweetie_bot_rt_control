#!/bin/sh
VARIABLE="{
header: auto,
name: ['leg1'],
pose: [
 {
  position: {
    x: 0.0411275,
    y: -0.0804284,
    z: 0.05 },
  orientation: {
    x: 0.707108,
    y: -3.44086e-05,
    z: -3.44085e-05,
    w: 0.707105 },
 }
]}"

# z: 0.0299998
echo "<<<" "$VARIABLE" "\n<<<"
echo ">>>"
rostopic pub /input_limbs_cartesian sweetie_bot_kinematics_msgs/CartesianState --once "$VARIABLE"
echo ">>>"
