#!/bin/sh
VARIABLE="
{
 header: auto,
 name: ['joint51', 'joint52', 'joint53', 'joint54'],
 position: [1.1, 1.2, 1.3, 1.4],
 velocity: [2.11, 2.12, 2.13, 2.14],
 effort:   [3.11, 3.12, 3.13, 3.14]
}"
echo "<<<" "$VARIABLE" "\n<<<"
echo ">>>"
rostopic pub /aggregator/input_joint_state sensor_msgs/JointState --once "$VARIABLE"
echo ">>>"
